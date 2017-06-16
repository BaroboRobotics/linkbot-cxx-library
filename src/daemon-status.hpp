#ifndef LINKBOT_DAEMON_STATUS_HPP
#define LINKBOT_DAEMON_STATUS_HPP

#include <daemon.pb.hpp>

#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <string>

#define LINKBOT_status(x) x = linkbot_daemon_Status_##x

enum class DaemonStatus {
    // Keep up to date with daemon.proto
    LINKBOT_status(OK),
    LINKBOT_status(CANNOT_OPEN_DONGLE),
    LINKBOT_status(DONGLE_NOT_FOUND),
    LINKBOT_status(PORT_OUT_OF_RANGE),
    LINKBOT_status(UNREGISTERED_SERIALID),
    LINKBOT_status(INVALID_SERIALID),
    LINKBOT_status(DAEMON_UNAVAILABLE),
    LINKBOT_status(STRANGE_DONGLE),
    LINKBOT_status(INCOMPATIBLE_FIRMWARE),
    LINKBOT_status(BUFFER_OVERFLOW),
    LINKBOT_status(OTHER_ERROR),
};

#undef LINKBOT_status

class DaemonErrorCategory: public boost::system::error_category {
public:
    virtual const char* name () const BOOST_NOEXCEPT override;
    virtual std::string message (int ev) const BOOST_NOEXCEPT override;
};

const boost::system::error_category& daemonErrorCategory();
boost::system::error_code make_error_code(DaemonStatus status);
boost::system::error_condition make_error_condition(DaemonStatus status);

namespace boost { namespace system {
template <> struct is_error_code_enum<::DaemonStatus>: public std::true_type { };
}}

#endif
