// Copyright (c) 2015-2017, XMOS Ltd, All rights reserved
#include "xc2compat.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <smi.h>
#include <xassert.h>
#include <print.h>
#include "xtcp.h"
/* Used to prevent conflict with lwIP */
#include "xtcp_lwip_includes.h"
#include "xtcp_shared.h"

#define MAXIMUM_NUMBER_OF_CLIENTS 32
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// These pointers are used to store connections for sending in
// xcoredev.xc
extern client interface ethernet_tx_if  * unsafe xtcp_i_eth_tx;
extern client interface mii_if * unsafe xtcp_i_mii;
extern mii_info_t xtcp_mii_info;

typedef enum client_state_t
{
  XTCP_NULL_STATE = 0,
  XTCP_CONNECT_PENDING = 1,
  XTCP_CONNECTED = 2,
  XTCP_LISTENING = 4
} client_state_t;

typedef struct client_data_buffer_t
{
  int client_num;
  int id;
  client_state_t state;
  struct pbuf * unsafe recv;
} client_data_buffer_t;

client_data_buffer_t client_data_buffers[MAXIMUM_NUMBER_OF_CLIENTS];

static void
xtcp_lwip_low_level_init(struct netif &netif, char mac_address[6])
{
  /* set MAC hardware address length */
  netif.hwaddr_len = ETHARP_HWADDR_LEN;
  /* set MAC hardware address */
  memcpy(netif.hwaddr, mac_address, ETHARP_HWADDR_LEN);
  /* maximum transfer unit */
  netif.mtu = 1500;
  /* device capabilities */
  netif.flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_UP;
}

static void
xtcp_lwip_init_timers(unsigned period[NUM_TIMEOUTS],
                      unsigned timeout[NUM_TIMEOUTS],
                      unsigned time_now)
{
  period[ARP_TIMEOUT] = ARP_TMR_INTERVAL * XS1_TIMER_KHZ;
  period[AUTOIP_TIMEOUT] = AUTOIP_TMR_INTERVAL * XS1_TIMER_KHZ;
  period[TCP_TIMEOUT] = TCP_TMR_INTERVAL * XS1_TIMER_KHZ;
  period[IGMP_TIMEOUT] = IGMP_TMR_INTERVAL * XS1_TIMER_KHZ;
  period[DHCP_COARSE_TIMEOUT] = DHCP_COARSE_TIMER_MSECS * XS1_TIMER_KHZ;
  period[DHCP_FINE_TIMEOUT] = DHCP_FINE_TIMER_MSECS * XS1_TIMER_KHZ;

  for (int i=0; i < NUM_TIMEOUTS; i++) {
    timeout[i] = time_now + period[i];
  }
}

static unsafe void
process_rx_packet(char buffer[], size_t n_bytes,
                  struct netif *unsafe netif)
{
  struct pbuf *unsafe p, *unsafe q;
  if (ETH_PAD_SIZE) {
    n_bytes += ETH_PAD_SIZE; /* allow room for Ethernet padding */
  }
  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, n_bytes, PBUF_POOL);

  if (p != NULL) {
    if (ETH_PAD_SIZE) {
      pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
    }
    /* We iterate over the pbuf chain until we have read the entire
     * packet into the pbuf. */
    unsigned byte_cnt = 0;
    for (q = p; q != NULL; q = q->next) {
      /* Read enough bytes to fill this pbuf in the chain. The
       * available data in the pbuf is given by the q->len
       * variable. */
      memcpy(q->payload, (char *unsafe)&buffer[byte_cnt], q->len);
      byte_cnt += q->len;
    }

    if (ETH_PAD_SIZE) {
      pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
    }

    ethernet_input(p, netif); // Process the packet
  } else {
    debug_printf("No buffers free\n");
  }
}

static inline unsafe void
remove_pcb_udp_connection(struct udp_pcb * unsafe pcb,
                          unsigned slot)
{
  pcb->connection_ports[slot] = 0;
  for (int i=0; i<4; i++)
    pcb->connection_addrs[slot][i] = 0;
}

static inline unsafe unsigned
pcb_contains_connection(struct udp_pcb * unsafe pcb,
                        unsigned char * unsafe addr,
                        unsigned port_number)
{
  for(int i=0; i<CONNECTIONS_PER_UDP_PORT; i++) {
    if(pcb->connection_ports[i] == port_number &&
       XTCP_IPADDR_CMP(pcb->connection_addrs[i], addr)) {
      return 1;
    }
  }
  return 0;
}

static inline unsafe int
first_empty_pcb_udp_slot(struct udp_pcb * unsafe pcb) {
  for(int i=0; i<CONNECTIONS_PER_UDP_PORT; i++) {
    if(pcb->connection_ports[i] == 0) {
      return i;
    }
  }
  return -1;
}

static unsafe unsigned
add_udp_connection(struct udp_pcb * unsafe pcb,
                   unsigned char * unsafe addr,
                   unsigned port_number)
{
  if(!pcb_contains_connection(pcb, (unsigned char * unsafe) addr, port_number)) {
    int empty_slot = first_empty_pcb_udp_slot(pcb);
    if(empty_slot != -1) {
      pcb->connection_ports[empty_slot] = port_number;
      for (int i=0; i<4; i++) {
        pcb->connection_addrs[empty_slot][i] = ((unsigned char * unsafe) addr)[i];
      }
      return 1;
    } else {
      debug_printf("Reached maximum amount of remote UDP connections per PCB");
    }
  }
  return 0;
}

static int find_client_data_buffer(int client_num, int id)
{
  for (int i = 0; i < MAXIMUM_NUMBER_OF_CLIENTS; ++i) {
    if (client_data_buffers[i].client_num == client_num &&
        client_data_buffers[i].id == id) {
      return i;
    }
  }

  return -1;
}

static int get_free_client_data_buffer(void)
{
  return find_client_data_buffer(-1, -1);
}

static void free_client_data_buffer(int index)
{
  if (index != -1) {
    client_data_buffers[index].id = -1;
    client_data_buffers[index].client_num = -1;

    if (client_data_buffers[index].recv != NULL) {
      pbuf_free(client_data_buffers[index].recv);
    }
  }
}

void
xtcp_lwip(server xtcp_if i_xtcp[n_xtcp],
          static const unsigned n_xtcp,
          client mii_if ?i_mii,
          client ethernet_cfg_if ?i_eth_cfg,
          client ethernet_rx_if ?i_eth_rx,
          client ethernet_tx_if ?i_eth_tx,
          client smi_if ?i_smi,
          uint8_t phy_address,
          const char (&?mac_address0)[6],
          otp_ports_t &?otp_ports,
          xtcp_ipconfig_t &ipconfig)
{
  unsafe {
    mii_info_t mii_info;
    timer timers[NUM_TIMEOUTS];
    unsigned timeout[NUM_TIMEOUTS];
    unsigned period[NUM_TIMEOUTS];

    char mac_address[6];
    struct netif my_netif;
    struct netif *unsafe netif;

    if (!isnull(mac_address0)) {
      memcpy(mac_address, mac_address0, 6);
    } else if (!isnull(otp_ports)) {
      otp_board_info_get_mac(otp_ports, 0, mac_address);
    } else if (!isnull(i_eth_cfg)) {
      i_eth_cfg.get_macaddr(0, mac_address);
    } else {
      fail("Must supply OTP ports or MAC address to xtcp component");
    }

    if (!isnull(i_mii)) {
      mii_info = i_mii.init();
      xtcp_mii_info = mii_info;
      xtcp_i_mii = (client mii_if * unsafe) &i_mii;
    }

    if (!isnull(i_eth_cfg)) {
      xtcp_i_eth_tx = (client ethernet_tx_if * unsafe) &i_eth_tx;
      i_eth_cfg.set_macaddr(0, mac_address);

      size_t index = i_eth_rx.get_index();
      ethernet_macaddr_filter_t macaddr_filter;
      memcpy(macaddr_filter.addr, mac_address, sizeof(mac_address));
      i_eth_cfg.add_macaddr_filter(index, 0, macaddr_filter);

      // Add broadcast filter
      for (size_t i = 0; i < 6; i++)
        macaddr_filter.addr[i] = 0xff;
      i_eth_cfg.add_macaddr_filter(index, 0, macaddr_filter);

      // Only allow ARP and IP packets to the stack
      i_eth_cfg.add_ethertype_filter(index, 0x0806);
      i_eth_cfg.add_ethertype_filter(index, 0x0800);
    }

    int using_fixed_ip = 0;
    for (int i = 0; i < sizeof(ipconfig.ipaddr); i++) {
      if (((unsigned char *)ipconfig.ipaddr)[i]) {
        using_fixed_ip = 1;
        break;
      }
    }

    lwip_init();
    xtcp_init_queue(n_xtcp, i_xtcp);

    ip4_addr_t ipaddr, netmask, gateway;
    memcpy(&ipaddr, ipconfig.ipaddr, sizeof(xtcp_ipaddr_t));
    memcpy(&netmask, ipconfig.netmask, sizeof(xtcp_ipaddr_t));
    memcpy(&gateway, ipconfig.gateway, sizeof(xtcp_ipaddr_t));

    for (int i = 0; i < MAXIMUM_NUMBER_OF_CLIENTS; ++i) {
      client_data_buffers[i].client_num = -1;
      client_data_buffers[i].id = -1;
    }

    netif = &my_netif;
    netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL);
    netif_set_default(netif);

    xtcp_lwip_low_level_init(my_netif, mac_address);

    if (ipconfig.ipaddr[0] == 0) {
      if (dhcp_start(netif) != ERR_OK) fail("DHCP error");
    }
    netif_set_up(netif);

    int time_now;
    timers[0] :> time_now;
    xtcp_lwip_init_timers(period, timeout, time_now);

    while (1) {
      select {
      case !isnull(i_mii) => mii_incoming_packet(mii_info):
        int * unsafe data;
        do {
          int nbytes;
          unsigned timestamp;
          {data, nbytes, timestamp} = i_mii.get_incoming_packet();
          if (data) {
            process_rx_packet((char *)data, nbytes, netif);
            i_mii.release_packet(data);
          }
        } while (data != NULL);
        break;

      case !isnull(i_eth_rx) => i_eth_rx.packet_ready():
        char buffer[MAX_PACKET_BYTES];
        ethernet_packet_info_t desc;
        i_eth_rx.get_packet(desc, (char *) buffer, MAX_PACKET_BYTES);

        if (desc.type == ETH_DATA) {
          process_rx_packet(buffer, desc.len, netif);
          if (!get_if_state()) netif_set_link_up(netif);
        }
        else if (isnull(i_smi) && desc.type == ETH_IF_STATUS) {
          if (((unsigned char *)buffer)[0] == ETHERNET_LINK_UP) {
            netif_set_link_up(netif);
          } else {
            netif_set_link_down(netif);
            xtcp_if_down();
          }
        }
        break;

      /* Client calls get_packet() after the server has notified with event_ready().
       * This function pops the event and updates with latest values */
      case i_xtcp[unsigned i].get_event(xtcp_connection_t &conn) -> xtcp_event_type_t event:
        client_queue_t head = dequeue_event(i);
        unsigned bytecount = 0;

        if (head.xtcp_conn->protocol == XTCP_PROTOCOL_TCP) {
          struct tcp_pcb *unsafe t_pcb = (struct tcp_pcb *unsafe) head.xtcp_conn->stack_conn;
          head.xtcp_conn->mss = tcp_mss(t_pcb);
        }

        event = head.xtcp_conn->event = head.xtcp_event;
        head.xtcp_conn->packet_length = bytecount;
        memcpy(&conn, head.xtcp_conn, sizeof(xtcp_connection_t));

        renotify(i);
        break;

      case i_xtcp[unsigned i].listen(xtcp_connection_t &conn, int port_number, xtcp_protocol_t protocol) -> int result:
        const int index = find_client_data_buffer(conn.client_num, conn.id);

        xassert(index != -1 && client_data_buffers[index].state != XTCP_LISTENING);

        if (protocol == XTCP_PROTOCOL_TCP) {
          struct tcp_pcb *unsafe pcb = tcp_new();
          tcp_bind(pcb, NULL, port_number);
          pcb = tcp_listen(pcb);
          pcb->xtcp_conn = conn;
          client_data_buffers[index].state = XTCP_LISTENING;
        } else {
          struct udp_pcb *unsafe pcb = udp_new();
          udp_bind(pcb, NULL, port_number);
          unsigned char blank_ip[4] = {0};
          memset(pcb->connection_ports, 0, sizeof(unsigned) * CONNECTIONS_PER_UDP_PORT);
          memset(pcb->connection_addrs, 0, sizeof(unsigned char) * CONNECTIONS_PER_UDP_PORT * 4);
          conn = pcb->xtcp_conn = fill_xtcp_state(conn, blank_ip, port_number, 0, pcb);
        }
        result = 0;
        break;

      /* Same as listen, but doesn't create the UDP PCB */
      case i_xtcp[unsigned i].bind_local_udp(const xtcp_connection_t &conn, unsigned port_number):
        if (conn.protocol == XTCP_PROTOCOL_TCP) break;

        struct udp_pcb *unsafe u_pcb = (struct udp_pcb * unsafe) conn.stack_conn;
        /* Rebind to new port */
        udp_bind(u_pcb, NULL, port_number);
        break;

      case i_xtcp[unsigned i].bind_remote_udp(const xtcp_connection_t &conn, xtcp_ipaddr_t ipaddr, unsigned port_number):
        if (conn.protocol == XTCP_PROTOCOL_TCP) break;
        struct udp_pcb *unsafe u_pcb = (struct udp_pcb * unsafe) conn.stack_conn;

        xtcp_ipaddr_t ip;
        memcpy(ip, ipaddr, sizeof(xtcp_ipaddr_t));
        unsigned port_n = port_number;

        err_t e = udp_connect(u_pcb, (struct ip_addr * unsafe) ip, port_n);
        if(e != ERR_OK)
          debug_printf("udp_connect() failed\n");
        add_udp_connection(u_pcb, ip, port_n);
        break;

      case i_xtcp[unsigned i].unlisten(unsigned port_number):
        /* Need to make sure we've found all associated PCBs with port */
        unsigned all_pcbs_found = 0;
        while(!all_pcbs_found) {
          struct tcp_pcb * unsafe t_pcb = xtcp_lookup_tcp_pcb_state_from_port(port_number);
          if (t_pcb) {
            tcp_abort(t_pcb);
          } else {
            struct udp_pcb * unsafe u_pcb = xtcp_lookup_udp_pcb_state_from_port(port_number);
            if (u_pcb) {
              udp_remove(u_pcb);
            } else {
              all_pcbs_found = 1;
            }
          }
        }
        break;

      case i_xtcp[unsigned i].close(const xtcp_connection_t &conn):
        const int index = find_client_data_buffer(conn.client_num, conn.id);
        xtcp_connection_t *unsafe xtcp_conn;

        if (index != -1) {
          client_data_buffers[index].client_num = -1;
          client_data_buffers[index].id = -1;

          if (conn.protocol == XTCP_PROTOCOL_TCP) {
            struct tcp_pcb *unsafe t_pcb = (struct tcp_pcb * unsafe) conn.stack_conn;
            xtcp_conn = &(t_pcb->xtcp_conn);
            tcp_close(t_pcb);
          } else if (conn.protocol == XTCP_PROTOCOL_UDP) {
            /* Take a local copy to pass to the functions */
            const xtcp_connection_t local_conn = conn;

            struct udp_pcb *unsafe u_pcb = xtcp_lookup_udp_pcb_state(local_conn.id);
            xtcp_conn = &(u_pcb->xtcp_conn);
            int slot = pcb_contains_connection(u_pcb, local_conn.remote_addr, local_conn.remote_port);
            if(slot != -1) {
              remove_pcb_udp_connection(u_pcb, slot);
            }
          }
          else {
            debug_printf("UNKNOWN PROTOCOL\n");
          }
          free_client_data_buffer(index);
          enqueue_event_and_notify(i, XTCP_CLOSED, xtcp_conn);
        }
        break;

      case i_xtcp[unsigned i].join_multicast_group(xtcp_ipaddr_t addr):
        ip4_addr_t group_addr;
        memcpy(&group_addr, &addr, sizeof(ip4_addr_t));
        igmp_joingroup(IPADDR_ANY, &group_addr);
        break;

      case i_xtcp[unsigned i].leave_multicast_group(xtcp_ipaddr_t addr):
        ip4_addr_t group_addr;
        memcpy(&group_addr, &addr, sizeof(ip4_addr_t));
        igmp_leavegroup(IPADDR_ANY, &group_addr);
        break;

      case i_xtcp[unsigned i].abort(const xtcp_connection_t &conn):
        const int index = find_client_data_buffer(conn.client_num, conn.id);
        xtcp_connection_t *unsafe xtcp_conn;
        xtcp_event_type_t event;

        if (conn.protocol == XTCP_PROTOCOL_TCP) {
          struct tcp_pcb *unsafe t_pcb = (struct tcp_pcb *unsafe) conn.stack_conn;
          xtcp_conn = &(t_pcb->xtcp_conn);
          tcp_abort(t_pcb);
          event = XTCP_ABORTED;
        } else {
          struct udp_pcb *unsafe u_pcb = (struct udp_pcb *unsafe) conn.stack_conn;
          xtcp_conn = &(u_pcb->xtcp_conn);
          int slot = pcb_contains_connection(u_pcb, xtcp_conn->remote_addr, xtcp_conn->remote_port);
          if(slot != -1) {
            remove_pcb_udp_connection(u_pcb, slot);
          }
          event = XTCP_CLOSED;
        }

        rm_recv_events(xtcp_conn->id, i);
        enqueue_event_and_notify(i, event, xtcp_conn);
        if (index != -1) {
          free_client_data_buffer(index);
        }
        break;

      case i_xtcp[unsigned i].socket(xtcp_protocol_t protocol) -> xtcp_connection_t result:
        result = create_xtcp_empty_state(i, protocol);

        const int index = get_free_client_data_buffer();
        client_data_buffers[index].client_num = result.client_num;
        client_data_buffers[index].id = result.id;
        client_data_buffers[index].recv = NULL;
        client_data_buffers[index].state = XTCP_NULL_STATE;
        break;

      case i_xtcp[unsigned i].connect(xtcp_connection_t &conn, unsigned short port_number, xtcp_ipaddr_t ipaddr) -> int result:
        xtcp_ipaddr_t ip;
        memcpy(ip, ipaddr, sizeof(xtcp_ipaddr_t));
        const int index = find_client_data_buffer(conn.client_num, conn.id);

        xassert(index != -1 && client_data_buffers[index].state != XTCP_CONNECT_PENDING);

        if (conn.protocol == XTCP_PROTOCOL_TCP) {
          struct tcp_pcb * unsafe const pcb = tcp_new();

          if (pcb != NULL) {
            const err_t err = tcp_connect(pcb, (struct ip_addr * unsafe)ip, port_number, NULL);

            if (err == ERR_OK) {
              pcb->xtcp_conn = conn;
              client_data_buffers[index].state = XTCP_CONNECT_PENDING;
              result = XTCP_EINPROGRESS;
            } else {
              result = XTCP_EHOSTUNREACH;
            }
          } else {
            result = XTCP_ENOBUFS;
          }
        } else {
          struct udp_pcb *unsafe pcb = udp_new();
          udp_bind(pcb, NULL, 0);
          unsigned char blank_ip[4] = {0,0,0,0};
          memset(pcb->connection_ports, 0, sizeof(unsigned) * CONNECTIONS_PER_UDP_PORT);
          memset(pcb->connection_addrs, 0, sizeof(unsigned char) * CONNECTIONS_PER_UDP_PORT * 4);
          pcb->remote_port = port_number;
          conn = pcb->xtcp_conn = fill_xtcp_state(conn, (unsigned char * unsafe) &pcb->remote_ip,
                                                  pcb->local_port, pcb->remote_port, pcb);
          if (add_udp_connection(pcb, ip, port_number)) {
            enqueue_event_and_notify(i, XTCP_NEW_CONNECTION, &(pcb->xtcp_conn));
            client_data_buffers[index].state = XTCP_CONNECTED;
            result = 0;
          }
        }
        break;

      case i_xtcp[unsigned i].send(xtcp_connection_t &conn, char data[], unsigned len) -> int result:
        const int index = find_client_data_buffer(conn.client_num, conn.id);
        struct tcp_pcb *unsafe t_pcb = (struct tcp_pcb *unsafe)conn.stack_conn;

        if (index != -1) {
          if (len > 0) {
            if (conn.protocol == XTCP_PROTOCOL_TCP) {
              char data_tmp[1000] = {};
              result = MIN(1000, len);
              memcpy(data_tmp, data, result);

              struct tcp_pcb * unsafe const t_pcb = (struct tcp_pcb * unsafe const)conn.stack_conn;
              const err_t error = tcp_write(t_pcb, data_tmp, result, TCP_WRITE_FLAG_COPY);
              if (error != ERR_OK) {
                debug_printf("error = %d\n", error);
                result = XTCP_ECONNRESET;
                tcp_abort(t_pcb);

                rm_recv_events(conn.id, i);
                if (index != -1) {
                  free_client_data_buffer(index);
                }

                enqueue_event_and_notify(t_pcb->xtcp_conn.client_num, XTCP_ABORTED, &(t_pcb->xtcp_conn));
              }
            } else {
              struct pbuf *unsafe new_pbuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
              result = MIN(1000, len);
              memcpy(new_pbuf->payload, data, len);

              struct udp_pcb * unsafe const u_pcb = (struct udp_pcb * unsafe const) conn.stack_conn;
              if (u_pcb->flags & UDP_FLAGS_CONNECTED) {
                const err_t error = udp_send(u_pcb, new_pbuf);
                xassert(error == ERR_OK);
              } else {
                const err_t error = udp_sendto(u_pcb, new_pbuf, (ip_addr_t * unsafe) u_pcb->xtcp_conn.remote_addr, u_pcb->xtcp_conn.remote_port);
                xassert(error == ERR_OK);
              }

              pbuf_free(new_pbuf);
            }
          } else {
            result = XTCP_EMSGSIZE;
          }
        } else {
          result = XTCP_ENOTCONN;
        }
        break;

      case i_xtcp[unsigned i].set_appstate(const xtcp_connection_t &conn, xtcp_appstate_t appstate):
        if (conn.protocol == XTCP_PROTOCOL_TCP) {
          struct tcp_pcb * unsafe t_pcb = (struct tcp_pcb * unsafe) conn.stack_conn;
          t_pcb->xtcp_conn.appstate = appstate;
        } else {
          struct udp_pcb * unsafe u_pcb = (struct udp_pcb * unsafe) conn.stack_conn;
          u_pcb->xtcp_conn.appstate = appstate;
        }
        break;

      case i_xtcp[unsigned i].request_host_by_name(const char hostname[], unsigned name_len):
        struct dns_table_entry *unsafe dns;
        int table_entry;
        if (name_len >= DNS_MAX_NAME_LENGTH)
          fail("DNS host name len exceeds DNS_MAX_NAME_LENGTH");
        dns = dns_find_entry(&table_entry);
        if(dns) {
          memcpy(dns->name, hostname, name_len+1);
          dns_enqueue(name_len, (void *)i, table_entry);
        }
        break;

      case i_xtcp[unsigned i].get_ipconfig(xtcp_ipconfig_t &ipconfig):
        memcpy(&ipconfig.ipaddr, &my_netif.ip_addr, sizeof(xtcp_ipaddr_t));
        memcpy(&ipconfig.netmask, &my_netif.netmask, sizeof(xtcp_ipaddr_t));
        memcpy(&ipconfig.gateway, &my_netif.gw, sizeof(xtcp_ipaddr_t));
        break;

      case i_xtcp[unsigned i].recv(xtcp_connection_t &conn, char buffer[], unsigned int length) -> int result:
        const int index = find_client_data_buffer(conn.client_num, conn.id);

        if (index != -1) {
          struct pbuf * const unsafe head = client_data_buffers[index].recv;

          if (head != NULL) {
            result = MIN(length, head->len);

            if (result != 0) {
              memcpy(buffer, head->payload, result);

              if (conn.protocol == XTCP_PROTOCOL_TCP) {
                struct tcp_pcb * unsafe const t_pcb = (struct tcp_pcb * unsafe const)conn.stack_conn;

                if (result < head->len) {
                  memmove(head->payload, head->payload + result, head->len - result);
                  head->len -= result;
                  head->tot_len -= result;
                } else {
                  pbuf_ref(head->next);
                  client_data_buffers[index].recv = pbuf_dechain(head);
                  pbuf_free(head);
                }

                if (client_data_buffers[index].recv != NULL) {
                  enqueue_event_and_notify(t_pcb->xtcp_conn.client_num, XTCP_RECV_DATA, &(t_pcb->xtcp_conn));
                }

                tcp_recved(t_pcb, result);
              } else {
                struct udp_pcb * unsafe const u_pcb = (struct udp_pcb * unsafe const) conn.stack_conn;

                if (result < head->len) {
                  memmove(head->payload, head->payload + result, head->len - result);
                  head->len -= result;
                  head->tot_len -= result;
                } else {
                  pbuf_ref(head->next);
                  client_data_buffers[index].recv = pbuf_dechain(head);
                  pbuf_free(head);
                }

                if (client_data_buffers[index].recv != NULL) {
                  enqueue_event_and_notify(u_pcb->xtcp_conn.client_num, XTCP_RECV_DATA, &(u_pcb->xtcp_conn));
                }
              }
            } else {
              result = XTCP_EAGAIN;
            }
          } else {
            result = XTCP_EAGAIN;
          }
        } else {
          result = XTCP_ENOTCONN;
        }
        break;
      case i_xtcp[unsigned i].is_ifup(void) -> int result:
        result = get_if_state();
        break;

      case(size_t i = 0; i < NUM_TIMEOUTS; i++)
        timers[i] when timerafter(timeout[i]) :> unsigned current:

        switch (i) {
        case ARP_TIMEOUT: {
          etharp_tmr();
          if (!isnull(i_smi)) {
            static int linkstate = 0;
            ethernet_link_state_t status = smi_get_link_state(i_smi, phy_address);
            if (!status && linkstate) {
              if (!isnull(i_eth_cfg))
                i_eth_cfg.set_link_state(0, status, LINK_100_MBPS_FULL_DUPLEX);
              xtcp_if_down();
              netif_set_link_down(netif);
            } else if (status && !linkstate) {
              if (!isnull(i_eth_cfg))
                i_eth_cfg.set_link_state(0, status, LINK_100_MBPS_FULL_DUPLEX);
              netif_set_link_up(netif);
            }
            linkstate = status;
          }

          if (!get_if_state() && netif_is_link_up(netif)) {
            if (dhcp_supplied_address(netif) ||
                using_fixed_ip) {
              xtcp_if_up();
            }
          }

          break;
        }
        case AUTOIP_TIMEOUT: autoip_tmr(); break;
        case TCP_TIMEOUT: tcp_tmr(); break;
        case IGMP_TIMEOUT: igmp_tmr(); break;
        case DHCP_COARSE_TIMEOUT: dhcp_coarse_tmr(); break;
        case DHCP_FINE_TIMEOUT: dhcp_fine_tmr(); break;
        default: fail("Bad timer\n"); break;
        }

        timeout[i] = current + period[i];
        break;

      default:
        //tcp_fasttmr();
        break;
      }
    }
  } /* Unsafe */
}

/* Function called by lwIP when any TCP event happens on a connection */
unsafe err_t
lwip_tcp_event(void *unsafe arg,
               struct tcp_pcb *unsafe pcb,
               enum lwip_event e,
               struct pbuf *unsafe p,
               u16_t size,
               err_t err)
{
  switch(e) {
    case LWIP_EVENT_ACCEPT:
      const int index = get_free_client_data_buffer();
      pcb->xtcp_conn = create_xtcp_empty_state(pcb->xtcp_conn.client_num, pcb->xtcp_conn.protocol);
      pcb->xtcp_conn = fill_xtcp_state(pcb->xtcp_conn, (unsigned char * unsafe) &pcb->remote_ip,
                                       pcb->local_port, pcb->remote_port, pcb);
      enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_NEW_CONNECTION,
                              &(pcb->xtcp_conn));

      if (index != -1) {
        client_data_buffers[index].client_num = pcb->xtcp_conn.client_num;
        client_data_buffers[index].id = pcb->xtcp_conn.id;
        client_data_buffers[index].recv = NULL;
        client_data_buffers[index].state = XTCP_CONNECTED;
      } else {
        tcp_close(pcb);
      }
      break;

    case LWIP_EVENT_CONNECTED:
      pcb->xtcp_conn = fill_xtcp_state(pcb->xtcp_conn, (unsigned char * unsafe) &pcb->remote_ip,
                                       pcb->local_port, pcb->remote_port, pcb);
      enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_NEW_CONNECTION,
                               &(pcb->xtcp_conn));

      const int index = find_client_data_buffer(pcb->xtcp_conn.client_num, pcb->xtcp_conn.id);
      client_data_buffers[index].recv = NULL;
      client_data_buffers[index].state = XTCP_CONNECTED;
      break;

    case LWIP_EVENT_RECV:
      if(p != NULL) {
        enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_RECV_DATA, &(pcb->xtcp_conn));

        const int index = find_client_data_buffer(pcb->xtcp_conn.client_num, pcb->xtcp_conn.id);
        if (index != -1) {
          if (client_data_buffers[index].recv == NULL) {
            pbuf_ref(p);
            client_data_buffers[index].recv = p;
          } else {
            pbuf_chain(client_data_buffers[index].recv, p);
          }
        }
      } else {
        enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_CLOSED, &(pcb->xtcp_conn));
      }
      break;

    case LWIP_EVENT_SENT:
      enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_SENT_DATA, &(pcb->xtcp_conn));
      break;

    case LWIP_EVENT_ERR:
      debug_printf("LWIP_EVENT_ERR: %s\n", lwip_strerr(err));
      break;
  }

  if (p != NULL) {
    pbuf_free(p);
  }

  return ERR_OK;
}

/* TODO: Make it so DNS can be called multiple times without dequeuing */
static xtcp_connection_t dns_dummy = {0};

/* Function called by lwIP when there is a DNS result */
unsafe void lwip_xtcpd_handle_dns_response(ip_addr_t * unsafe ipaddr, int client_num)
{
  for (int i=0; i<4; i++)
    dns_dummy.remote_addr[i] = ((unsigned char * unsafe) ipaddr)[i];
  enqueue_event_and_notify(client_num, XTCP_DNS_RESULT, &dns_dummy);
}

/* Function called by lwIP when any UDP event happens on a connection */
unsafe void udp_recv_event(void * unsafe arg,
                           struct udp_pcb * unsafe pcb,
                           struct pbuf * unsafe p,
                           const ip_addr_t * unsafe addr,
                           u16_t _port) /* The underscore prefix is added by xtcp_lwip_includes */
{
  switch (_port) {
    case DHCP_SERVER_PORT:
      dhcp_recv(arg, pcb, p, addr, _port);
      break;
    case DNS_SERVER_PORT:
      dns_recv(arg, pcb, p, addr, _port);
      break;
    default:
      if (pcb == NULL) {
        pbuf_free(p);
        break;
      } else {
        /* Update xtcp_conn */
        pcb->xtcp_conn.remote_port = _port;
        for (int i=0; i<4; i++) {
          pcb->xtcp_conn.remote_addr[i] = ((unsigned char * unsafe) addr)[i];
        }

        if(add_udp_connection(pcb, (unsigned char * unsafe) addr, _port)) {
          enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_NEW_CONNECTION, &(pcb->xtcp_conn));

          const int index = get_free_client_data_buffer();
          client_data_buffers[index].client_num = pcb->xtcp_conn.client_num;
          client_data_buffers[index].id = pcb->xtcp_conn.id;
          client_data_buffers[index].recv = NULL;
          client_data_buffers[index].state = XTCP_CONNECTED;
        }

        if (p != NULL) {
          enqueue_event_and_notify(pcb->xtcp_conn.client_num, XTCP_RECV_DATA, &(pcb->xtcp_conn));

          const int index = find_client_data_buffer(pcb->xtcp_conn.client_num, pcb->xtcp_conn.id);
          if (index != -1) {
            if (client_data_buffers[index].recv == NULL) {
              pbuf_ref(p);
              client_data_buffers[index].recv = p;
            } else {
              pbuf_chain(client_data_buffers[index].recv, p);
            }
          }
        }
      }
      pbuf_free(p);
      break;
  }
}
