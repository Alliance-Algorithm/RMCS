#include "utility/shared/interprocess.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

TEST(shm, shm) {
    using Client = rmcs::shm::Client<std::uint64_t>::Send;

    constexpr auto shm_name = "/rmcs_auto_aim_test";
    constexpr auto shm_size = rmcs::shm::Client<std::uint64_t>::context_len;

    // Create and size the shared memory object
    int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ASSERT_NE(fd, -1);
    ASSERT_EQ(ftruncate(fd, shm_size), 0);
    close(fd);

    auto client = Client {};
    ASSERT_TRUE(client.open(shm_name));
    ASSERT_TRUE(client.opened());

    // Optional cleanup
    shm_unlink(shm_name);
}
