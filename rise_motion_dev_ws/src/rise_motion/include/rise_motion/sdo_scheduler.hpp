#pragma once

#include "rise_motion/result.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

class SdoScheduler 
{
    public:

        using JobID = std::uint64_t;

        static constexpr JobID INVALID_JOB_ID = 0;

        enum class Operation 
        {
            READ,
            WRITE,
        };

        // returned by ec manager after attempting an SDO call
        enum class AttemptStatus 
        {
            SUCCESS,
            RETRYABLE_FAILURE,
            PERMANENT_FAILURE,
        };

        enum class ErrorCode
        {
            RETRY_LIMIT_REACHED,
            PERMANENT_FAILURE,
            CANCELLED,
            QUEUE_FULL,
        };

        struct Error 
        {
            SdoScheduler::ErrorCode code;
            std::string message = "";
        };

        using SdoResult = rise::Result<std::vector<std::uint8_t>, SdoScheduler::Error>;

        struct RetryOptions 
        {
            // 0 = retry infinitely
            std::size_t max_attempts{3};

            bool retry{false};
        };

        struct Request 
        {
            Operation operation{Operation::READ};

            std::uint16_t device_id{0};
            std::uint16_t index{0};
            std::uint8_t subindex{0};

            // Used only for READ jobs
            std::size_t read_size{0};

            // Used only for WRITE jobs
            std::vector<std::uint8_t> write_value{};
        };

        struct AttemptResult 
        {
            AttemptStatus status{AttemptStatus::PERMANENT_FAILURE};
            std::vector<std::uint8_t> value{};
        };

        struct Submission 
        {
            JobID id{INVALID_JOB_ID};
            std::future<SdoResult> future;
        };

        struct ActiveJob 
        {
            JobID id{INVALID_JOB_ID};
            Request request{};
        };


        explicit SdoScheduler(std::size_t max_queue_size = 64);
        ~SdoScheduler();

        SdoScheduler(const SdoScheduler &) = delete;
        SdoScheduler &operator=(const SdoScheduler &) = delete;


        static RetryOptions get_default_retry_options();

        Submission enqueue_read(
            std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::size_t read_size, 
            SdoScheduler::RetryOptions retry_options = SdoScheduler::get_default_retry_options());

        Submission enqueue_write(
            std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::vector<std::uint8_t> value, 
            SdoScheduler::RetryOptions retry_options = SdoScheduler::get_default_retry_options());

        
        std::optional<ActiveJob> get_job();

        bool complete_attempt(JobID id, AttemptResult attempt_result);

        bool cancel(JobID id);

        void cancel_all();

        std::size_t num_jobs_pending() const;


    private:

        struct Job 
        {
            JobID id{INVALID_JOB_ID};
            Request request{};
            RetryOptions retry_options{};

            std::size_t attempts{0};

            bool cancellation_requested{false};

            std::promise<SdoResult> promise;
        };

        Submission submit(Request request, RetryOptions retry_options);

        static SdoResult get_cancelled_result();

        mutable std::mutex mutex_;

        std::deque<std::shared_ptr<Job>> queue_;
        std::shared_ptr<Job> active_job_;

        std::size_t max_queue_size_;
        JobID next_id_{1};
};