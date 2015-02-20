#ifndef NET_TCP_H
#define NET_TCP_H

struct net_tcp_conn_s;

enum net_tcp_conn_state_e
{
  NET_TCP_CONNECTING,
  NET_TCP_CONNECTED,
  NET_TCP_PEER_CLOSED,
  NET_TCP_CLOSING,
};

struct net_tcp_handler_s
{
  void (*conn_req)(struct net_layer_s *tcp, struct net_tcp_conn_info_s *conn_info);
};

struct net_tcp_conn_handler_s
{
  void (*data)(struct net_tcp_conn_s *conn, const void *data, size_t size);
  void (*urg)(struct net_tcp_conn_s *conn, const void *data, size_t size);
  void (*closed)(struct net_tcp_conn_s *conn);
  void (*error)(struct net_tcp_conn_s *conn);
};

error_t net_tcp_create(
    struct net_layer_s *upper,
    const struct net_tcp_handler_s *handler,
    struct net_layer_s **tcp);

error_t net_tcp_connect(
    struct net_layer_s *tcp,
    const struct net_tcp_conn_handler_s *handler,
    struct net_tcp_conn_s **conn);

error_t net_tcp_accept(
    struct net_layer_s *tcp,
    const struct net_tcp_conn_handler_s *handler,
    const struct net_tcp_conn_info_s *ci,
    struct net_tcp_conn_s **conn);

error_t net_tcp_conn_destroy(
    struct net_tcp_conn_s *conn);

error_t net_tcp_conn_shutdown(
    struct net_tcp_conn_s *conn,
    bool_t shut_send, bool_t shut_recv);

error_t net_tcp_conn_write(
    struct net_tcp_conn_s *conn,
    const void *data, size_t size);

#endif
