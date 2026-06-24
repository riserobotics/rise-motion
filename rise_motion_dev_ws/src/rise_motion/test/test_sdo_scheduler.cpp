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


TEST(SdoSchedulerTest, FULL_QUEUE)
{
    SdoScheduler scheduler{1};

    auto first_submission = scheduler.enqueue_read(1, 0x6064, 0x00, 1);

    ASSERT_NE(first_submission.id, SdoScheduler::INVALID_JOB_ID);

    auto second_submission = scheduler.enqueue_read(1, 0x6064, 0x00, 1);

    EXPECT_EQ(second_submission.id, SdoScheduler::INVALID_JOB_ID);

    ASSERT_EQ(second_submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = second_submission.future.get();

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error.code, SdoScheduler::ErrorCode::QUEUE_FULL);
    EXPECT_EQ(scheduler.num_jobs_pending(), 1);
}


TEST(SdoSchedulerTest, CANCEL_JOB)
{
    SdoScheduler scheduler;

    auto submission = scheduler.enqueue_read(1, 0x6064, 0x00, 1);

    ASSERT_NE(submission.id, SdoScheduler::INVALID_JOB_ID);
    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    ASSERT_TRUE(scheduler.cancel(submission.id));

    ASSERT_EQ(submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = submission.future.get();

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error.code, SdoScheduler::ErrorCode::CANCELLED);
    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}


TEST(SdoSchedulerTest, CANCEL_ACTIVE_JOB)
{
    SdoScheduler scheduler;

    auto submission = scheduler.enqueue_read(1, 0x6064, 0x00, 1);

    ASSERT_NE(submission.id, SdoScheduler::INVALID_JOB_ID);
    EXPECT_EQ(scheduler.num_jobs_pending(), 1);

    auto job = scheduler.get_job();

    ASSERT_TRUE(job.has_value());

    ASSERT_TRUE(scheduler.cancel(submission.id));

    std::vector<std::uint8_t> reply = {0x11};

    bool completed = scheduler.complete_attempt(job->id, SdoScheduler::AttemptResult{SdoScheduler::AttemptStatus::SUCCESS, reply});

    ASSERT_TRUE(completed);

    ASSERT_EQ(submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto result = submission.future.get();

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error.code, SdoScheduler::ErrorCode::CANCELLED);
    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}

TEST(SdoSchedulerTest, CANCEL_ALL)
{
    SdoScheduler scheduler;

    auto first_submission = scheduler.enqueue_read(1, 0x6064, 0x00, 4);
    auto second_submission = scheduler.enqueue_write(1, 0x6060, 0x00, {0x08});

    ASSERT_NE(first_submission.id, SdoScheduler::INVALID_JOB_ID);
    ASSERT_NE(second_submission.id, SdoScheduler::INVALID_JOB_ID);

    EXPECT_EQ(scheduler.num_jobs_pending(), 2);

    scheduler.cancel_all();

    ASSERT_EQ(first_submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);
    ASSERT_EQ(second_submission.future.wait_for(std::chrono::milliseconds{1}), std::future_status::ready);

    const auto first_result = first_submission.future.get();
    const auto second_result = second_submission.future.get();

    ASSERT_FALSE(first_result);
    ASSERT_FALSE(second_result);

    EXPECT_EQ(first_result.error.code, SdoScheduler::ErrorCode::CANCELLED);
    EXPECT_EQ(second_result.error.code, SdoScheduler::ErrorCode::CANCELLED);

    EXPECT_EQ(scheduler.num_jobs_pending(), 0);
}