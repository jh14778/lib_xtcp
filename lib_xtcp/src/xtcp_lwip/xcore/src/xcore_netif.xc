#include "ethernet.h"
#include "mii.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"

client interface ethernet_tx_if  * unsafe xtcp_i_eth_tx = NULL;
client interface mii_if * unsafe xtcp_i_mii = NULL;
mii_info_t xtcp_mii_info;

unsafe err_t xcore_igmp_mac_filter(struct netif *unsafe netif,
                                   const ip4_addr_t *unsafe group,
                                   u8_t action) {
  return ERR_OK;
}

err_t xcore_linkoutput(struct netif *unsafe netif, struct pbuf *unsafe p) {
  static int txbuf[(ETHERNET_MAX_PACKET_SIZE+3)/4];
  static int tx_buf_in_use=0;

  if (tx_buf_in_use) {
    select {
    case mii_packet_sent(xtcp_mii_info):
      break;
    }
  }

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  struct pbuf *unsafe q;
  unsigned byte_cnt = 0;
  unsafe {
    for (q = p; q != NULL; q = q->next) {
      memcpy((char *unsafe)&txbuf[byte_cnt], q->payload, q->len);
      byte_cnt += q->len;
    }

    if (byte_cnt < 60) {
      for (int i=byte_cnt; i<60; i++)
        (txbuf, unsigned char[])[i] = 0;
        byte_cnt = 60;
    }

    xtcp_i_mii->send_packet(txbuf, byte_cnt);
  }
  tx_buf_in_use=1;

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  return ERR_OK;
}


void xcoredev_send(void) {};