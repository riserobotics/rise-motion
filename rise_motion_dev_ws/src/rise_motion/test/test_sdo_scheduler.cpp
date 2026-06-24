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


TEST(SdoSchedulerTest, RETRYABLE_FAILURE)
{
    SdoScheduler scheduler;

    auto retry_options = SdoScheduler::get_default_retry_options();
    retry_options.max_attempts = 2;

    auto submission = scheduler.enqueue_read(1, 0x6064, 0x00, 1, retry_options);

    ASSERT_NE(submission.id, SdoScheduler::INVALID_JOB_ID);
    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    auto first_job = scheduler.get_job();

    ASSERT_TRUE(first_job.has_value());

    auto first_reply = SdoScheduler::AttemptResult{SdoScheduler::AttemptStatus::RETRYABLE_FAILURE};
    bool first_completed = scheduler.complete_attempt(first_job->id, first_reply);

    ASSERT_TRUE(first_completed);
    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    auto second_job = scheduler.get_job();

    ASSERT_TRUE(second_job.has_value());
    EXPECT_EQ(second_job->id, submission.id);

    auto second_reply = SdoScheduler::AttemptResult{SdoScheduler::AttemptStatus::SUCCESS, {0x11}};
    bool second_completed = scheduler.complete_attempt(first_job->id, second_reply);

    ASSERT_TRUE(second_completed);
    ASSERT_EQ(submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = submission.future.get();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value, std::vector<std::uint8_t>{0x11});

    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}