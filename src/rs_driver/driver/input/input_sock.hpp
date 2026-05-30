
#pragma once

#ifdef ENABLE_EPOLL_RECEIVE
#include <rs_driver/driver/input/unix/input_sock_epoll.hpp>
#else
#include <rs_driver/driver/input/unix/input_sock_select.hpp>
#endif
