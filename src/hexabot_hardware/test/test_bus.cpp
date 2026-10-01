#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "hexabot_hardware/sts_bus.hpp"

using namespace hexabot_hardware;

namespace
{
std::unique_ptr<StsBus> makeBus(SimulatedBus ** raw)
{
  auto sim = std::make_unique<SimulatedBus>(std::vector<uint8_t>{1, 2, 3, 19, 20}, 1e6);
  *raw = sim.get();
  auto bus = std::make_unique<StsBus>(std::move(sim));
  EXPECT_TRUE(bus->open());
  return bus;
}
}  // namespace

TEST(StsBus, PingFindsOnlyPresentServos)
{
  SimulatedBus * sim;
  auto bus = makeBus(&sim);
  EXPECT_TRUE(bus->ping(1));
  EXPECT_TRUE(bus->ping(20));
  EXPECT_FALSE(bus->ping(7, 2));
}

TEST(StsBus, TorqueAndGoalPositionsMoveServos)
{
  SimulatedBus * sim;
  auto bus = makeBus(&sim);
  ASSERT_TRUE(bus->writeGoalPositions({{1, 1000}, {2, 3000}}));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  bus->readStates({1, 2}, 5);
  EXPECT_EQ(sim->position(1), 2048);   // no torque, no motion
  ASSERT_TRUE(bus->setTorque({1, 2}, true));
  EXPECT_TRUE(sim->torque(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  const auto st = bus->readStates({1, 2}, 5);
  ASSERT_EQ(st.size(), 2u);
  EXPECT_EQ(st.at(1).position, 1000);
  EXPECT_EQ(st.at(2).position, 3000);
  EXPECT_NEAR(st.at(1).voltage, 12.1, 1e-9);
}

TEST(StsBus, ReadStatesDecodesSignedLoadAndReportsMissing)
{
  SimulatedBus * sim;
  auto bus = makeBus(&sim);
  sim->setLoad(3, -420);
  sim->dropServo(19);
  const auto st = bus->readStates({1, 3, 19, 20}, 5);
  EXPECT_EQ(st.size(), 3u);
  EXPECT_EQ(st.at(3).load, -420);
  EXPECT_EQ(st.count(19), 0u);
}

TEST(StsBus, UnicastReadAndWrite)
{
  SimulatedBus * sim;
  auto bus = makeBus(&sim);
  ASSERT_TRUE(bus->write(2, sts::kTorqueEnable, {1}));
  EXPECT_TRUE(sim->torque(2));
  auto v = bus->read(2, sts::kPresentVoltage, 2);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ((*v)[0], 121);
  EXPECT_EQ((*v)[1], 34);
}
