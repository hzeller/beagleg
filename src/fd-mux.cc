// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#include "fd-mux.h"

#include <vector>

#include <sys/select.h>

bool FDMultiplexer::RunOnReadable(int fd, const Handler &handler) {
    return handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::IsRegisteredReadable(int fd) const {
    return handlers_.find(fd) != handlers_.end();
}

void FDMultiplexer::Loop() {
    fd_set read_fds;
    for (;;) {
        int maxfd = -1;
        FD_ZERO(&read_fds);
        for (const auto &it : handlers_) {
            if (it.first >= maxfd) maxfd = it.first+1;
            FD_SET(it.first, &read_fds);
        }
        if (maxfd < 0) {
            // file descriptors only can be registred from within handlers
            // or before running the Loop(). If no filedesctiptors are left,
            // there is no chance for any to re-appear, so we can exit.
            fprintf(stderr, "No filedescriptor registered. Exiting loop()");
            break;
        }

        int fds_ready = select(maxfd, &read_fds, nullptr, nullptr, nullptr);
        if (fds_ready < 0) {
            perror("select() failed");
            break;
        }

        if (fds_ready == 0) {
            // Timeout situation. We are not registering timeout handlers
            // currently.
            continue;
        }

        std::vector<int> to_delete;
        for (const auto &it : handlers_) {
            if (FD_ISSET(it.first, &read_fds)) {
                const bool retrigger = it.second();
                if (!retrigger) {
                    to_delete.push_back(it.first);
                }
                if (--fds_ready == 0)
                    break;
            }
        }
        for (int i : to_delete) {
            handlers_.erase(i);
        }
    }
}
