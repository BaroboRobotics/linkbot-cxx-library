#include "daemon-status.hpp"

const char* DaemonErrorCategory::name() const BOOST_NOEXCEPT {
    return "linkbotd";
}

std::string DaemonErrorCategory::message(int ev) const BOOST_NOEXCEPT {
    switch (DaemonStatus(ev)) {
#define ITEM(x) case DaemonStatus::x: return #x
        ITEM(OK);
        ITEM(CANNOT_OPEN_DONGLE);
        ITEM(DONGLE_NOT_FOUND);
        ITEM(PORT_OUT_OF_RANGE);
        ITEM(UNREGISTERED_SERIALID);
        ITEM(INVALID_SERIALID);
        ITEM(DAEMON_UNAVAILABLE);
        ITEM(STRANGE_DONGLE);
        ITEM(INCOMPATIBLE_FIRMWARE);
        ITEM(BUFFER_OVERFLOW);
        ITEM(OTHER_ERROR);
#undef ITEM
        default: return "(unknown status)";
    }
}

const boost::system::error_category& daemonErrorCategory() {
    static DaemonErrorCategory instance;
    return instance;
}

boost::system::error_code make_error_code(DaemonStatus status) {
    return boost::system::error_code(static_cast<int>(status),
        daemonErrorCategory());
}

boost::system::error_condition make_error_condition(DaemonStatus status) {
    return boost::system::error_condition(static_cast<int>(status),
        daemonErrorCategory());
}
