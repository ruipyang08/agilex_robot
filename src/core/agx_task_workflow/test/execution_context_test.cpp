#include <cassert>
#include <atomic>
#include <future>
#include <limits>
#include <stdexcept>
#include "agx_task_workflow/execution_context.hpp"

using namespace agx_task_workflow;

int main() {
  double now = 10.0;
  bool cancel = false;
  bool fault = false;
  int progress = 0;
  ExecutionContext context(2.0, [&] { return now; }, [&] { return cancel; },
      [&](const std::string&) { ++progress; }, [&] { return fault; });
  assert(context.Check() == FailureKind::None);
  now = 11.9;
  assert(context.ReportProgress("MOVING") == FailureKind::None);
  assert(progress == 1);
  now = 12.0;
  assert(context.Check() == FailureKind::Deadline);
  now = 11.0;
  assert(context.Check() == FailureKind::Deadline); // Terminal cause stays latched.
  assert(context.ReportProgress("LATE") == FailureKind::Deadline);
  assert(progress == 1);

  now = 20.0;
  ExecutionContext interrupted(5.0, [&] { return now; }, [&] { return cancel; },
      [&](const std::string&) { cancel = true; });
  assert(interrupted.ReportProgress("BOUNDARY") == FailureKind::Cancelled);
  cancel = false;
  assert(interrupted.Check() == FailureKind::Cancelled);

  ExecutionContext clock_fault(5.0, [&] { return now; }, [] { return false; });
  now = 19.0;
  assert(clock_fault.Check() == FailureKind::Fault);
  now = 20.0;
  assert(clock_fault.Check() == FailureKind::Fault);

  ExecutionContext invalid_clock(5.0, [&] { return now; }, [] { return false; });
  now = std::numeric_limits<double>::quiet_NaN();
  assert(invalid_clock.Check() == FailureKind::Fault);
  now = 20.0;
  ExecutionContext faulted(5.0, [&] { return now; }, [] { return false; }, {},
      [&] { return fault; });
  fault = true;
  assert(faulted.Check() == FailureKind::Fault);
  ExecutionContext callback_deadline(1.0, [&] { return now; }, [] { return false; },
      [&](const std::string&) { now += 1.0; });
  assert(callback_deadline.ReportProgress("BOUNDARY") == FailureKind::Deadline);
  fault = false;
  bool callback_ran = false;
  ExecutionContext callback_fault(5.0, [&] { return now; }, [] { return false; },
      [&](const std::string&) { callback_ran = true; fault = true; }, [&] { return fault; });
  assert(callback_fault.ReportProgress("BOUNDARY") == FailureKind::Fault);
  assert(callback_ran);
  fault = false;
  assert(faulted.Check() == FailureKind::Fault);
  bool rejected = false;
  try { ExecutionContext bad(0.0, [&] { return now; }, [] { return false; }); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);

  // A positive duration must advance the representable steady deadline.
  now=100.0;rejected=false;
  try { ExecutionContext tiny(std::numeric_limits<double>::denorm_min(), [&] {return now;}, [] {return false;}); }
  catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
  const double one_tick=std::nextafter(now,std::numeric_limits<double>::infinity())-now;
  ExecutionContext smallest(one_tick,[&] {return now;},[] {return false;});
  assert(smallest.Check()==FailureKind::None);
  now+=one_tick;assert(smallest.Check()==FailureKind::Deadline);
  now=std::numeric_limits<double>::max();rejected=false;
  try { ExecutionContext overflow(now,[&] {return now;},[] {return false;}); }
  catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);

  // A slow cancellation source must not make an earlier sampled timestamp
  // look like clock regression after a concurrent health check completes.
  std::atomic<int> ticks{100}, checks{0};
  std::promise<void> first_entered, release_first;
  auto released = release_first.get_future().share();
  ExecutionContext concurrent(100.0, [&] { return double(ticks++); }, [&] {
    if (checks++ == 0) { first_entered.set_value(); released.wait(); }
    return false;
  });
  auto first = std::async(std::launch::async, [&] { return concurrent.Check(); });
  first_entered.get_future().wait();
  auto second = std::async(std::launch::async, [&] { return concurrent.Check(); });
  assert(second.get() == FailureKind::None);
  release_first.set_value();
  assert(first.get() == FailureKind::None);
}
