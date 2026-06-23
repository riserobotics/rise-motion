#include <rise_motion/sdo_scheduler.hpp>

#include <algorithm>
#include <utility>

SdoScheduler::SdoScheduler(std::size_t max_queue_size) : max_queue_size_(max_queue_size) {}

SdoScheduler::~SdoScheduler() 
{
    cancel_all();
}


SdoScheduler::RetryOptions SdoScheduler::get_default_retry_options() 
{
    RetryOptions options;
    options.max_attempts = 3;
    options.retry = true;
    return options;
}

SdoScheduler::Submission SdoScheduler::enqueue_read(
    std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::size_t read_size) 
{
    return enqueue_read(device_id, index, subindex, read_size, get_default_retry_options());
}

SdoScheduler::Submission SdoScheduler::enqueue_read(
    std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::size_t read_size, 
    RetryOptions retry_options) 
{
    Request request;
    request.operation = Operation::READ;
    request.device_id = device_id;
    request.index = index;
    request.subindex = subindex;
    request.read_size = read_size;

    return submit(std::move(request), retry_options);
}

SdoScheduler::Submission SdoScheduler::enqueue_write(
    std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::vector<std::uint8_t> value) 
{
    return enqueue_write(device_id, index, subindex, std::move(value), get_default_retry_options());
}

SdoScheduler::Submission SdoScheduler::enqueue_write(
    std::uint16_t device_id, std::uint16_t index, std::uint8_t subindex, std::vector<std::uint8_t> value,
    RetryOptions retry_options) 
{
    Request request;
    request.operation = Operation::WRITE;
    request.device_id = device_id;
    request.index = index;
    request.subindex = subindex;
    request.write_value = std::move(value);

    return submit(std::move(request), retry_options);
}

std::optional<SdoScheduler::ActiveJob> SdoScheduler::get_job() 
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (active_job_) {
        return std::nullopt;
    }

    if (queue_.empty()) {
        return std::nullopt;
    }

    const auto ready = queue_.begin();

    active_job_ = *ready;
    queue_.erase(ready);

    ++active_job_->attempts;

    return ActiveJob{active_job_->id, active_job_->request};
}

bool SdoScheduler::complete_attempt(JobID id, AttemptResult attempt_result) 
{
    std::shared_ptr<Job> completed_job;
    std::optional<SdoResult> completed_result;

    std::lock_guard<std::mutex> lock(mutex_);

    if (!active_job_ || active_job_->id != id) {
        return false;
    }

    std::shared_ptr<Job> job = std::move(active_job_);
    active_job_.reset();

    if (job->cancellation_requested) {
        completed_job = job;
        completed_result = get_cancelled_result();
    } 
    else if (attempt_result.status == AttemptStatus::SUCCESS) {
        completed_job = job;
        completed_result = SdoResult::ok(std::move(attempt_result.value));

    } 
    else if (attempt_result.status == AttemptStatus::PERMANENT_FAILURE) {
        completed_job = job;
        completed_result = SdoResult::err({ErrorCode::PERMANENT_FAILURE, "The SDO request failed permanently. No retry."});
    } 
    else {
        const bool retry_limit_reached = 
            job->retry_options.max_attempts != 0 && job->attempts >= job->retry_options.max_attempts;

        const bool retry_allowed =
            job->retry_options.retry && !retry_limit_reached;

        if (retry_allowed) {
            // put it at the end so a failed job cannot block other jobs
            queue_.push_front(job);
        } 
        else {
            completed_job = job;

            completed_result = SdoResult::err({ErrorCode::RETRY_LIMIT_REACHED, 
                "The SDO request could not be successfully completed in the given number of attempts"});
        }
    }
    
    if (completed_job) {
        completed_job->promise.set_value(std::move(*completed_result));
    }

    return true;
}

bool SdoScheduler::cancel(JobID id) 
{
    std::shared_ptr<Job> cancelled_job;
  
    std::lock_guard<std::mutex> lock(mutex_);

    const auto queued_it = std::find_if(queue_.begin(), queue_.end(), 
        [id](const std::shared_ptr<Job> &job) {return job->id == id;});

    if (queued_it != queue_.end()) {
        cancelled_job = *queued_it;
        queue_.erase(queued_it);
    } 
    else if (active_job_ && active_job_->id == id) {
        active_job_->cancellation_requested = true;
        return true;
    } 
    else {
        return false;
    }
  
    cancelled_job->promise.set_value(get_cancelled_result());
    return true;
}

void SdoScheduler::cancel_all() 
{
    std::vector<std::shared_ptr<Job>> cancelled_jobs;
  
    std::lock_guard<std::mutex> lock(mutex_);

    cancelled_jobs.assign(queue_.begin(), queue_.end());
    queue_.clear();

    if (active_job_) {
      active_job_->cancellation_requested = true;
    }

    for (const auto &job : cancelled_jobs) {
        job->promise.set_value(get_cancelled_result());
    }
}

std::size_t SdoScheduler::num_jobs_pending() const 
{
  std::lock_guard<std::mutex> lock(mutex_);

  return queue_.size() + (active_job_ ? 1 : 0);
}

SdoScheduler::Submission SdoScheduler::submit(Request request, RetryOptions retry_options) 
{
    auto job = std::make_shared<Job>();

    job->request = std::move(request);
    job->retry_options = retry_options;

    auto future = job->promise.get_future();

    std::lock_guard<std::mutex> lock(mutex_);

    if (queue_.size() + (active_job_ ? 1 : 0) >= max_queue_size_) {
        job->promise.set_value(SdoResult::err({ErrorCode::QUEUE_FULL, "The queue is full. SDO request rejected."}));

        return Submission{INVALID_JOB_ID, std::move(future)};
    } 
    else {
        job->id = next_id_++;

        if (next_id_ == INVALID_JOB_ID) {
            next_id_ = 1;
        }

        queue_.push_back(job);
    }

    return Submission{job->id, std::move(future)};
}

SdoScheduler::SdoResult SdoScheduler::get_cancelled_result() 
{
    return SdoScheduler::SdoResult::err({ErrorCode::CANCELLED, "The SDO request got cancelled"});
}