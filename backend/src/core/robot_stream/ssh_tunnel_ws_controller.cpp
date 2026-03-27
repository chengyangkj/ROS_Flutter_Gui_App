#include "core/robot_stream/ssh_tunnel_ws_controller.hpp"

#include "common/config/config.hpp"
#include "common/logger/logger.h"

#include <drogon/WebSocketConnection.h>
#include <netdb.h>
#include <sys/socket.h>
#include <trantor/net/InetAddress.h>
#include <trantor/net/TcpClient.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace ros_gui_backend {

namespace {

bool ResolveToInetAddress(const std::string& host, uint16_t port, trantor::InetAddress* out) {
  struct addrinfo hints {};
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_family = AF_UNSPEC;
  addrinfo* res = nullptr;
  const std::string ps = std::to_string(static_cast<unsigned>(port));
  const int gai = getaddrinfo(host.c_str(), ps.c_str(), &hints, &res);
  if (gai != 0 || res == nullptr) {
    if (res != nullptr) {
      freeaddrinfo(res);
    }
    return false;
  }
  bool ok = false;
  for (addrinfo* p = res; p != nullptr; p = p->ai_next) {
    if (p->ai_addr == nullptr) {
      continue;
    }
    if (p->ai_family == AF_INET) {
      *out = trantor::InetAddress(*reinterpret_cast<sockaddr_in*>(p->ai_addr));
      ok = true;
      break;
    }
    if (p->ai_family == AF_INET6) {
      *out = trantor::InetAddress(*reinterpret_cast<sockaddr_in6*>(p->ai_addr));
      ok = true;
      break;
    }
  }
  freeaddrinfo(res);
  return ok;
}

struct SshTunnelSession {
  std::weak_ptr<drogon::WebSocketConnection> ws;
  std::shared_ptr<trantor::TcpClient> tcpClient;
  trantor::TcpConnectionPtr tcp;
  std::mutex pendingMu;
  std::vector<std::string> pendingFromWs;

  void flushPending() {
    trantor::TcpConnectionPtr t;
    {
      std::lock_guard<std::mutex> lock(pendingMu);
      t = tcp;
      if (!t) {
        return;
      }
    }
    std::vector<std::string> batch;
    {
      std::lock_guard<std::mutex> lock(pendingMu);
      batch = std::move(pendingFromWs);
    }
    for (auto& chunk : batch) {
      t->send(std::move(chunk));
    }
  }
};

}  // namespace

void SshTunnelWsController::handleNewConnection(
    const drogon::HttpRequestPtr&, const drogon::WebSocketConnectionPtr& conn) {
  const auto& g = Config::Instance()->GuiApp();
  if (g.SshHost.empty() || g.SshPort <= 0 || g.SshPort > 65535) {
    conn->shutdown(drogon::CloseCode::kViolation, "SSH target not configured");
    return;
  }
  const auto port = static_cast<uint16_t>(g.SshPort);
  trantor::InetAddress peer;
  if (!ResolveToInetAddress(g.SshHost, port, &peer)) {
    LOGGER_WARN("ssh tunnel: resolve failed host={} port={}", g.SshHost, g.SshPort);
    conn->shutdown(drogon::CloseCode::kViolation, "SSH resolve failed");
    return;
  }

  auto sess = std::make_shared<SshTunnelSession>();
  sess->ws = conn;

  auto loop = drogon::app().getLoop();
  if (loop == nullptr) {
    conn->shutdown(drogon::CloseCode::kUnexpectedCondition, "no event loop");
    return;
  }

  auto tcpClient = std::make_shared<trantor::TcpClient>(loop, peer, "ssh_ws_tun");
  sess->tcpClient = tcpClient;

  tcpClient->setConnectionErrorCallback([sess]() {
    if (auto ws = sess->ws.lock()) {
      ws->shutdown(drogon::CloseCode::kUnexpectedCondition, "SSH TCP failed");
    }
  });

  tcpClient->setConnectionCallback([sess](const trantor::TcpConnectionPtr& tcpConn) {
    if (!tcpConn->connected()) {
      std::lock_guard<std::mutex> lock(sess->pendingMu);
      sess->tcp.reset();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(sess->pendingMu);
      sess->tcp = tcpConn;
    }
    tcpConn->setRecvMsgCallback([sess](const trantor::TcpConnectionPtr&, trantor::MsgBuffer* buf) {
      const size_t n = buf->readableBytes();
      if (n == 0) {
        buf->retrieveAll();
        return;
      }
      std::string data(buf->peek(), n);
      buf->retrieveAll();
      if (auto ws = sess->ws.lock()) {
        ws->send(std::move(data), drogon::WebSocketMessageType::Binary);
      }
    });
    tcpConn->setCloseCallback([sess](const trantor::TcpConnectionPtr&) {
      if (auto ws = sess->ws.lock()) {
        ws->forceClose();
      }
    });
    sess->flushPending();
  });

  conn->setContext(sess);
  tcpClient->connect();
  LOGGER_INFO("ssh tunnel ws peer={} -> {}:{}", conn->peerAddr().toIpPort(), g.SshHost, g.SshPort);
}

void SshTunnelWsController::handleNewMessage(
    const drogon::WebSocketConnectionPtr& conn, std::string&& message, const drogon::WebSocketMessageType& type) {
  if (type != drogon::WebSocketMessageType::Binary) {
    return;
  }
  if (!conn->hasContext()) {
    return;
  }
  auto sess = conn->getContext<SshTunnelSession>();
  if (!sess) {
    return;
  }
  trantor::TcpConnectionPtr t;
  {
    std::lock_guard<std::mutex> lock(sess->pendingMu);
    t = sess->tcp;
    if (!t) {
      sess->pendingFromWs.push_back(std::move(message));
      return;
    }
  }
  t->send(std::move(message));
}

void SshTunnelWsController::handleConnectionClosed(const drogon::WebSocketConnectionPtr& conn) {
  if (!conn->hasContext()) {
    return;
  }
  auto sess = conn->getContext<SshTunnelSession>();
  if (sess && sess->tcpClient) {
    sess->tcpClient->stop();
    sess->tcpClient.reset();
  }
  conn->clearContext();
}

}  // namespace ros_gui_backend
