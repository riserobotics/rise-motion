#include <chrono>
#include <cstdint>
#include <future>
#include <vector>

#include <gtest/gtest.h>
#include "rise_motion/sdo_scheduler.hpp"


TEST(SdoSchedulerTest, SUCCESSFUL_READ) 
{
    SdoScheduler scheduler;

    auto submission = scheduler.enqueue_read(1, 0x6064, 0x00, 4);

    ASSERT_NE(submission.id, SdoScheduler::INVALID_JOB_ID);

    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    auto job = scheduler.get_job();

    ASSERT_TRUE(job.has_value());
    EXPECT_EQ(job->id, submission.id);
    EXPECT_EQ(job->request.operation, SdoScheduler::Operation::READ);
    EXPECT_EQ(job->request.device_id, 1);
    EXPECT_EQ(job->request.index, 0x6064);
    EXPECT_EQ(job->request.subindex, 0x00);
    EXPECT_EQ(job->request.read_size, 4);

    std::vector<std::uint8_t> reply = {0x11, 0x22, 0x33, 0x44};

    bool completed = scheduler.complete_attempt(job->id, SdoScheduler::AttemptResult{SdoScheduler::AttemptStatus::SUCCESS, reply});

    ASSERT_TRUE(completed);

    ASSERT_EQ(submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = submission.future.get();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value[0], 0x11);
    EXPECT_EQ(result.value[1], 0x22);
    EXPECT_EQ(result.value[2], 0x33);
    EXPECT_EQ(result.value[3], 0x44);

    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}


TEST(SdoSchedulerTest, SUCCESSFUL_WRITE) 
{
    SdoScheduler scheduler;

    auto submission = scheduler.enqueue_write(1, 0x6064, 0x00, {0xff});

    ASSERT_NE(submission.id, SdoScheduler::INVALID_JOB_ID);

    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    auto job = scheduler.get_job();

    ASSERT_TRUE(job.has_value());
    EXPECT_EQ(job->id, submission.id);
    EXPECT_EQ(job->request.operation, SdoScheduler::Operation::WRITE);
    EXPECT_EQ(job->request.device_id, 1);
    EXPECT_EQ(job->request.index, 0x6064);
    EXPECT_EQ(job->request.subindex, 0x00);
    EXPECT_EQ(job->request.write_value, std::vector<std::uint8_t>{0xff});

    bool completed = scheduler.complete_attempt(job->id, SdoScheduler::AttemptResult{SdoScheduler::AttemptStatus::SUCCESS});

    ASSERT_TRUE(completed);

    ASSERT_EQ(submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = submission.future.get();

    ASSERT_TRUE(result);

    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}
