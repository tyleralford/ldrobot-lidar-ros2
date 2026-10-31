#include <gtest/gtest.h>

#include "data_watchdog.hpp"

using ldlidar::DataWatchdog;
using State = DataWatchdog::State;

TEST(DataWatchdog, IsOkWhileScansArrive)
{
  DataWatchdog dog(5.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(dog.update(false, true, i * 0.1), State::kOk);
  }
}

TEST(DataWatchdog, ATimeoutIsMissingAndFailsOnlyAfterTheLimit)
{
  DataWatchdog dog(5.0);
  EXPECT_EQ(dog.update(true, false, 10.0), State::kMissing);
  EXPECT_TRUE(dog.missing());
  EXPECT_EQ(dog.update(true, false, 14.9), State::kMissing);
  EXPECT_EQ(dog.update(true, false, 15.0), State::kFailed);
}

TEST(DataWatchdog, AWaitingPassNeitherStartsNorEndsAnOutage)
{
  DataWatchdog dog(5.0);
  EXPECT_EQ(dog.update(false, false, 1.0), State::kOk);          // waiting for the next turn
  EXPECT_EQ(dog.update(true, false, 2.0), State::kMissing);
  EXPECT_EQ(dog.update(false, false, 3.0), State::kMissing);     // still waiting: still missing
  EXPECT_EQ(dog.update(true, false, 7.0), State::kFailed);       // counted from the first one
}

TEST(DataWatchdog, ARecoveryForgetsTheOutage)
{
  DataWatchdog dog(5.0);
  dog.update(true, false, 0.0);
  EXPECT_EQ(dog.update(false, true, 4.0), State::kOk);
  EXPECT_FALSE(dog.missing());
  EXPECT_EQ(dog.update(true, false, 6.0), State::kMissing);      // a new outage, from 6 s
  EXPECT_EQ(dog.update(true, false, 10.9), State::kMissing);
  EXPECT_EQ(dog.update(true, false, 11.0), State::kFailed);
}

TEST(DataWatchdog, ALimitOfZeroNeverFails)
{
  DataWatchdog dog(0.0);
  EXPECT_EQ(dog.update(true, false, 0.0), State::kMissing);
  EXPECT_EQ(dog.update(true, false, 1e9), State::kMissing);
}
