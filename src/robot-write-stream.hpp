// In order to instantiate a `composed::rpc_client<>` for the linkbot.dongle interface, we need a
// suitable `composed::rpc_stream<>`-like type which implements an `async_write` in terms of the
// linkbot.daemon.transmit RPC request.
//
// This is that `composed::rpc_stream<>`-like type.

#ifndef LINKBOT_ROBOT_WRITE_STREAM_HPP
#define LINKBOT_ROBOT_WRITE_STREAM_HPP

#include <daemon.pb.hpp>
#include <robot.pb.hpp>

#include "daemon-status.hpp"

#include <composed/op.hpp>

#include <beast/core/handler_alloc.hpp>

#include <boost/asio/io_service.hpp>

#include <exception>
#include <string>
#include <chrono>

#include <boost/asio/yield.hpp>

template <class DaemonTransactor>
class RobotWriteStream {
public:
    RobotWriteStream(DaemonTransactor& t, const std::string& id)
        : next_layer_(t)
        , serialId(id)
    {
        if (serialId.size() > 4) {
            throw boost::system::system_error{make_error_code(DaemonStatus::INVALID_SERIALID)};
        }
    }

    boost::asio::io_service& get_io_service() { return next_layer_.get_io_service(); }
    auto& next_layer() { return next_layer_; }
    const auto& next_layer() const { return next_layer_; }
    auto& lowest_layer() { return next_layer_.lowest_layer(); }
    const auto& lowest_layer() const { return next_layer_.lowest_layer(); }

private:
    template <class Handler = void(boost::system::error_code)>
    struct TransmitOp;

public:
    template <class T, class Token>
    auto async_write(const T& message, Token&& token) {
        linkbot_daemon_transmit_In transmitRequest{};
        transmitRequest.has_destination = true;
        strcpy(transmitRequest.destination.value, serialId.c_str());
        transmitRequest.has_payload = true;
        nanopb::assign(transmitRequest.payload.arg, message);
        composed::operation<TransmitOp<>>{}(*this, transmitRequest, std::forward<Token>(token));
    }

private:
    DaemonTransactor& next_layer_;
    std::string serialId;
};

template <class DaemonTransactor>
template <class Handler>
struct RobotWriteStream<DaemonTransactor>::TransmitOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;

    RobotWriteStream& self;
    linkbot_daemon_transmit_In transmitRequest;

    composed::associated_logger_t<handler_type> lg;
    boost::system::error_code ec;

    TransmitOp(handler_type& h, RobotWriteStream& s, const linkbot_daemon_transmit_In& tr)
        : self(s)
        , transmitRequest(tr)
        , lg(composed::get_associated_logger(h))
    {}

    void operator()(composed::op<TransmitOp>& op);
};

template <class DaemonTransactor>
template <class Handler>
void RobotWriteStream<DaemonTransactor>::TransmitOp<Handler>::operator()(composed::op<TransmitOp>& op) {
    if (!ec) reenter(this) {
        using namespace std::literals;
        yield return self.next_layer_.async_do_request(transmitRequest, linkbot_daemon_RpcReply_transmit_tag,
                5s, op(ec, std::ignore));
        if (self.next_layer_.reply().transmit.has_status && self.next_layer_.reply().transmit.status) {
            ec = make_error_code(DaemonStatus(self.next_layer_.reply().transmit.status));
        }
    }
    op.complete(ec);
}

#include <boost/asio/unyield.hpp>

#endif