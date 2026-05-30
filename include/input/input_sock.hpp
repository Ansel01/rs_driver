
#pragma once

#ifdef ENABLE_EPOLL_RECEIVE
#include <input/input_sock_epoll.hpp>
#else
#include <input/input_sock_select.hpp>
#endif
