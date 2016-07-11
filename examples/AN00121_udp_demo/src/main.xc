// Copyright (c) 2016, XMOS Ltd, All rights reserved

#include <platform.h>
#include <string.h>
#include "debug_print.h"
#include <xtcp.h>

#include "otp_board_info.h"
#include "ethernet.h"
#include "smi.h"

#define EXPLORER_KIT    0
#define SLICEKIT_L16    0
#define MIC_ARRAY       1

#if SLICEKIT_L16
// Here are the port definitions required by ethernet. This port assignment
// is for the L16 sliceKIT with the ethernet slice plugged into the
// CIRCLE slot.
port p_eth_rxclk  = on tile[1]: XS1_PORT_1J;
port p_eth_rxd    = on tile[1]: XS1_PORT_4E;
port p_eth_txd    = on tile[1]: XS1_PORT_4F;
port p_eth_rxdv   = on tile[1]: XS1_PORT_1K;
port p_eth_txen   = on tile[1]: XS1_PORT_1L;
port p_eth_txclk  = on tile[1]: XS1_PORT_1I;
port p_eth_int    = on tile[1]: XS1_PORT_1O;
port p_eth_rxerr  = on tile[1]: XS1_PORT_1P;
port p_eth_timing = on tile[1]: XS1_PORT_8C;

clock eth_rxclk   = on tile[1]: XS1_CLKBLK_1;
clock eth_txclk   = on tile[1]: XS1_CLKBLK_2;

// SMI
port p_smi_mdio = on tile[1]: XS1_PORT_1M;
port p_smi_mdc  = on tile[1]: XS1_PORT_1N;

// These ports are for accessing the OTP memory
otp_ports_t otp_ports = on tile[1]: OTP_PORTS_INITIALIZER;

#elif EXPLORER_KIT

// eXplorerKIT RGMII port map
otp_ports_t otp_ports = on tile[0]: OTP_PORTS_INITIALIZER;

rgmii_ports_t rgmii_ports = on tile[1]: RGMII_PORTS_INITIALIZER;

port p_smi_mdio   = on tile[1]: XS1_PORT_1C;
port p_smi_mdc    = on tile[1]: XS1_PORT_1D;
port p_eth_reset  = on tile[1]: XS1_PORT_1N;

#define XTCP_MII_BUFSIZE (4096)
#define ETHERNET_SMI_PHY_ADDRESS (0)
#define POLL_INTERVAL 600000000

#elif MIC_ARRAY

// Microphone array reference design
port p_eth_rxclk  = on tile[1]: XS1_PORT_1A;
port p_eth_rxd    = on tile[1]: XS1_PORT_4A;
port p_eth_txd    = on tile[1]: XS1_PORT_4B;
port p_eth_rxdv   = on tile[1]: XS1_PORT_1C;
port p_eth_txen   = on tile[1]: XS1_PORT_1D;
port p_eth_txclk  = on tile[1]: XS1_PORT_1B;
port p_eth_rxerr  = on tile[1]: XS1_PORT_1K;
port p_eth_timing = on tile[1]: XS1_PORT_8C;

clock eth_rxclk   = on tile[1]: XS1_CLKBLK_1;
clock eth_txclk   = on tile[1]: XS1_CLKBLK_2;

// SMI
port p_smi        = on tile[1]: XS1_PORT_4C; // Bit 0: MDC, Bit 1: MDIO

// OTP
otp_ports_t otp_ports = on tile[1]: OTP_PORTS_INITIALIZER;

port p_rst_shared = on tile[1]: XS1_PORT_4F; // Bit 0: DAC_RST_N, Bit 1: ETH_RST_N

#else

#error "Unknown board"

#endif

#if 0
// IP Config - change this to suit your network.  Leave with all
// 0 values to use DHCP/AutoIP
xtcp_ipconfig_t ipconfig = {
        { 0, 0, 0, 0 }, // ip address (eg 192,168,0,2)
        { 0, 0, 0, 0 }, // netmask (eg 255,255,255,0)
        { 0, 0, 0, 0 } // gateway (eg 192,168,0,1)
};
#else
xtcp_ipconfig_t ipconfig = {
        { 172, 16, 0, 2 }, // ip address (eg 192,168,0,2)
        { 255, 255, 255, 0 }, // netmask (eg 255,255,255,0)
        { 0, 0, 0, 0 } // gateway (eg 192,168,0,1)
};
#endif

// Defines
#define RX_BUFFER_SIZE 1500
#define INCOMING_PORT 15533
#define BROADCAST_INTERVAL 600000000
#define BROADCAST_PORT 15534
#define BROADCAST_ADDR {255,255,255,255}
#define BROADCAST_MSG "XMOS Broadcast\n"
#define INIT_VAL -1

enum flag_status {TRUE=1, FALSE=0};

#if EXPLORER_KIT
// An enum to manage the array of connections from the ethernet component
// to its clients.
enum eth_clients {
  ETH_TO_XTCP,
  NUM_ETH_CLIENTS
};

enum cfg_clients {
  CFG_TO_XTCP,
  CFG_TO_PHY_DRIVER,
  NUM_CFG_CLIENTS
};

[[combinable]]
void ar8035_phy_driver(client interface smi_if smi,
                client interface ethernet_cfg_if eth) {
  ethernet_link_state_t link_state = ETHERNET_LINK_DOWN;
  //ethernet_speed_t link_speed = LINK_1000_MBPS_FULL_DUPLEX;
  ethernet_speed_t link_speed = LINK_100_MBPS_FULL_DUPLEX;
  const int phy_reset_delay_ms = 1;
  const int link_poll_period_ms = 1000;
  const int phy_address = 0x4;
  timer tmr;
  int t;
  tmr :> t;
  p_eth_reset <: 0;
  delay_milliseconds(phy_reset_delay_ms);
  p_eth_reset <: 1;

  while (smi_phy_is_powered_down(smi, phy_address));
  //smi_configure(smi, phy_address, LINK_1000_MBPS_FULL_DUPLEX, SMI_ENABLE_AUTONEG);
  smi_configure(smi, phy_address, LINK_100_MBPS_FULL_DUPLEX, SMI_ENABLE_AUTONEG);

  while (1) {
    select {
    case tmr when timerafter(t) :> t:
      ethernet_link_state_t new_state = smi_get_link_state(smi, phy_address);
      // Read AR8035 status register bits 15:14 to get the current link speed
      if (new_state == ETHERNET_LINK_UP) {
        link_speed = (ethernet_speed_t)(smi.read_reg(phy_address, 0x11) >> 14) & 3;
      }
      if (new_state != link_state) {
        link_state = new_state;
        eth.set_link_state(0, new_state, link_speed);
      }
      t += link_poll_period_ms * XS1_TIMER_KHZ;
      break;
    }
  }
}

#endif  //EXPLORER_KIT

/** Simple UDP reflection thread.
 *
 * This thread does two things:
 *
 *   - Reponds to incoming packets on port INCOMING_PORT and
 *     with a packet with the same content back to the sender.
 *   - Periodically sends out a fixed packet to a broadcast IP address.
 *
 */
void udp_reflect(chanend c_xtcp)
{
  xtcp_connection_t conn;  // A temporary variable to hold
                           // connections associated with an event
  xtcp_connection_t responding_connection; // The connection to the remote end
                                           // we are responding to
  xtcp_connection_t broadcast_connection; // The connection out to the broadcast
                                          // address
  xtcp_ipaddr_t broadcast_addr = BROADCAST_ADDR;
  int send_flag = TRUE;
  // int send_flag = FALSE;  // This flag is set when the thread is in the
                      // middle of sending a response packet
  int broadcast_send_flag = FALSE; // This flag is set when the thread is in the
                               // middle of sending a broadcast packet
  timer tmr;
  unsigned int time;

  // The buffers for incoming data, outgoing responses and outgoing broadcast
  // messages
  char rx_buffer[RX_BUFFER_SIZE];
  char tx_buffer[RX_BUFFER_SIZE];
  char broadcast_buffer[RX_BUFFER_SIZE] = BROADCAST_MSG;

  int response_len;  // The length of the response the thread is sending
  int broadcast_len; // The length of the broadcast message the thread is
                     // sending


  // Maintain track of two connections. Initially they are not initialized
  // which can be represented by setting their ID to -1
  responding_connection.id = INIT_VAL;
  broadcast_connection.id = INIT_VAL;

  // Instruct server to listen and create new connections on the incoming port
  xtcp_listen(c_xtcp, INCOMING_PORT, XTCP_PROTOCOL_TCP);

  tmr :> time;
  while (1) {
    select {

    // Respond to an event from the tcp server
    case xtcp_event(c_xtcp, conn):
      switch (conn.event)
        {
        case XTCP_IFUP:
          // When the interface goes up, set up the broadcast connection.
          // This connection will persist while the interface is up
          // and is only used for outgoing broadcast messages
          xtcp_connect(c_xtcp,
                       BROADCAST_PORT,
                       broadcast_addr,
                       XTCP_PROTOCOL_UDP);
          debug_printf("IFUP\n");
          break;

        case XTCP_IFDOWN:
          // Tidy up and close any connections we have open
          if (responding_connection.id != INIT_VAL) {
            xtcp_close(c_xtcp, responding_connection);
            responding_connection.id = INIT_VAL;
          }
          if (broadcast_connection.id != INIT_VAL) {
            xtcp_close(c_xtcp, broadcast_connection);
            broadcast_connection.id = INIT_VAL;
          }
          debug_printf("IFDOWN\n");
          break;

        case XTCP_NEW_CONNECTION:

          // The tcp server is giving us a new connection.
          // It is either a remote host connecting on the listening port
          // or the broadcast connection the threads asked for with
          // the xtcp_connect() call
          if (XTCP_IPADDR_CMP(conn.remote_addr, broadcast_addr)) {
            // This is the broadcast connection
            debug_printf("New broadcast connection established: %d\n", conn.id);
            broadcast_connection = conn;
         }
          else {
            // This is a new connection to the listening port
            debug_printf("New connection to listening port: %d\n", conn.local_port);
            if (responding_connection.id == INIT_VAL) {
              responding_connection = conn;
            }
            else {
              debug_printf("Cannot handle new connection\n");
              xtcp_close(c_xtcp, conn);
            }
          }
          break;

        case XTCP_RECV_DATA:
          // When we get a packet in:
          //
          //  - fill the tx buffer
          //  - initiate a send on that connection
          //
          response_len = xtcp_recv_count(c_xtcp, rx_buffer, RX_BUFFER_SIZE);
          debug_printf("Got data: %d bytes\n", response_len);

          for (int i=0;i<response_len;i++)
            tx_buffer[i] = rx_buffer[i];

          if (!send_flag) {
            xtcp_init_send(c_xtcp, conn);
            send_flag = TRUE;
            debug_printf("Responding\n");
          }
          else {
            // Cannot respond here since the send buffer is being used
          }
          break;

      case XTCP_REQUEST_DATA:
      case XTCP_RESEND_DATA:
        // The tcp server wants data, this may be for the broadcast connection
        // or the reponding connection

        if (conn.id == broadcast_connection.id) {
          xtcp_send(c_xtcp, broadcast_buffer, broadcast_len);
        }
        else {
          xtcp_send(c_xtcp, tx_buffer, response_len);
        }
        break;

      case XTCP_SENT_DATA:
        xtcp_complete_send(c_xtcp);
        if (conn.id == broadcast_connection.id) {
          // When a broadcast message send is complete the connection is kept
          // open for the next one
          debug_printf("Sent Broadcast\n");
          broadcast_send_flag = FALSE;
        }
        else {
          // When a reponse is sent, the connection is closed opening up
          // for another new connection on the listening port
          debug_printf("Sent Response\n");
          send_flag = FALSE;
        }
        break;

      case XTCP_TIMED_OUT:
      case XTCP_ABORTED:
      case XTCP_CLOSED:
        debug_printf("Closed connection: %d\n", conn.id);
        xtcp_close(c_xtcp, conn);
        responding_connection.id = INIT_VAL;
        break;

      case XTCP_ALREADY_HANDLED:
          break;
      }
      break;

    // This is the periodic case, it occurs every BROADCAST_INTERVAL
    // timer ticks
    case tmr when timerafter(time + BROADCAST_INTERVAL) :> void:

      // A broadcast message can be sent if the connection is established
      // and one is not already being sent on that connection
      if (broadcast_connection.id != INIT_VAL && !broadcast_send_flag)  {
        debug_printf("Sending broadcast message\n");
        broadcast_len = strlen(broadcast_buffer);
        xtcp_init_send(c_xtcp, broadcast_connection);
        broadcast_send_flag = TRUE;
      }
      tmr :> time;
      break;
    }
  }
}

#define XTCP_MII_BUFSIZE (4096)
#define ETHERNET_SMI_PHY_ADDRESS (0)

#if SLICEKIT_L16 || MIC_ARRAY
int main(void) {
  chan c_xtcp[1];
  mii_if i_mii;
  smi_if i_smi;
  par {
    // MII/ethernet driver
    on tile[1]: mii(i_mii, p_eth_rxclk, p_eth_rxerr, p_eth_rxd, p_eth_rxdv,
                    p_eth_txclk, p_eth_txen, p_eth_txd, p_eth_timing,
                    eth_rxclk, eth_txclk, XTCP_MII_BUFSIZE)

#if SLICEKIT_L16
    // SMI/ethernet phy driver
    on tile[1]: smi(i_smi, p_smi_mdio, p_smi_mdc);
#else // MIC_ARRAY
    on tile[1]: [[distribute]] smi_singleport(i_smi, p_smi, 1, 0);
#endif

    // TCP component
    on tile[1]: {
#if MIC_ARRAY
      p_rst_shared <: 0xF;
#endif

      xtcp(c_xtcp, 1, i_mii,
                     null, null, null,
                     i_smi, ETHERNET_SMI_PHY_ADDRESS,
                     null, otp_ports, ipconfig);
    }

    // The simple udp reflector thread
    on tile[0]: udp_reflect(c_xtcp[0]);

  }
  return 0;
}

#else
/* eXplorerKIT */

int main(void) {
  chan c_xtcp[1];
  ethernet_cfg_if i_cfg[NUM_CFG_CLIENTS];
  ethernet_rx_if i_rx[NUM_ETH_CLIENTS];
  ethernet_tx_if i_tx[NUM_ETH_CLIENTS];
  streaming chan c_rgmii_cfg;
  smi_if i_smi;

  par {

    on tile[1]: rgmii_ethernet_mac(i_rx, NUM_ETH_CLIENTS,
                                         i_tx, NUM_ETH_CLIENTS,
                                         null, null,
                                         c_rgmii_cfg,
                                         rgmii_ports,
                                         ETHERNET_DISABLE_SHAPER);

    on tile[1].core[0]: rgmii_ethernet_mac_config(i_cfg, NUM_CFG_CLIENTS, c_rgmii_cfg);

    on tile[1].core[0]: ar8035_phy_driver(i_smi, i_cfg[CFG_TO_PHY_DRIVER]);

    on tile[1]: smi(i_smi, p_smi_mdio, p_smi_mdc);

    on tile[0]: xtcp(c_xtcp, 1, null,
                      i_cfg[CFG_TO_XTCP], i_rx[ETH_TO_XTCP], i_tx[ETH_TO_XTCP],
                      null, ETHERNET_SMI_PHY_ADDRESS,
                      null, otp_ports, ipconfig);

    // The simple udp reflector thread
    on tile[0]: udp_reflect(c_xtcp[0]);

  }
  return 0;
} //EXPLORER_KIT

#endif
