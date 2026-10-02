#include <gtest/gtest.h>

#include <cmath>

#include "hexabot_hardware/sts_protocol.hpp"

using namespace hexabot_hardware::sts;

TEST(StsProtocol, PingPacket)
{
  // Example from the Feetech manual: ping servo 1.
  EXPECT_EQ(makePing(1), (Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(StsProtocol, ReadPacket)
{
  // Read 2 bytes of present position (0x38) from servo 1.
  EXPECT_EQ(makeRead(1, kPresentPosition, 2),
    (Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(StsProtocol, WriteGoalPositionLittleEndian)
{
  const Bytes p = makeWrite(1, kGoalPosition, u16(2048));
  // FF FF 01 05 03 2A 00 08 CHK
  ASSERT_EQ(p.size(), 9u);
  EXPECT_EQ(p[5], 0x2A);
  EXPECT_EQ(p[6], 0x00);
  EXPECT_EQ(p[7], 0x08);
  EXPECT_EQ(p[8], static_cast<uint8_t>(~(0x01 + 0x05 + 0x03 + 0x2A + 0x00 + 0x08) & 0xFF));
}

TEST(StsProtocol, SyncWriteLayout)
{
  const Bytes p = makeSyncWrite(kGoalPosition, 2, {{1, u16(100)}, {2, u16(4000)}});
  ASSERT_EQ(p.size(), 5u + 2u + 2u * 3u + 1u);
  EXPECT_EQ(p[2], kBroadcastId);
  EXPECT_EQ(p[3], 2 + 2 + 6);       // LEN = params + 2
  EXPECT_EQ(p[4], kSyncWrite);
  EXPECT_EQ(p[5], kGoalPosition);
  EXPECT_EQ(p[6], 2);
  EXPECT_EQ(p[7], 1);
  EXPECT_EQ(readU16(&p[8]), 100);
  EXPECT_EQ(p[10], 2);
  EXPECT_EQ(readU16(&p[11]), 4000);
}

TEST(StsProtocol, SyncReadLayout)
{
  const Bytes p = makeSyncRead(kPresentPosition, 8, {1, 2, 3});
  EXPECT_EQ(p, (Bytes{0xFF, 0xFF, 0xFE, 0x07, 0x82, 0x38, 0x08, 0x01, 0x02, 0x03,
      static_cast<uint8_t>(~(0xFE + 0x07 + 0x82 + 0x38 + 0x08 + 1 + 2 + 3) & 0xFF)}));
}

TEST(StsProtocol, ParserHandlesNoiseSplitsAndBadChecksums)
{
  StatusParser parser;
  // Status of servo 5 with two data bytes: FF FF 05 04 00 34 12 CHK
  Bytes good{0xFF, 0xFF, 0x05, 0x04, 0x00, 0x34, 0x12};
  good.push_back(checksum(0x05, 0x04, 0x00, &good[5], 2));
  Bytes bad = good;
  bad.back() ^= 0x55;
  Bytes stream{0x00, 0xFF, 0x13};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.push_back(0xFF);   // stray byte before a header: FF FF FF ...
  stream.insert(stream.end(), good.begin(), good.end());
  parser.feed(stream.data(), 5);   // split delivery
  EXPECT_FALSE(parser.next().has_value());
  parser.feed(stream.data() + 5, stream.size() - 5);
  auto p = parser.next();
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->id, 5);
  EXPECT_EQ(p->error, 0);
  EXPECT_EQ(readU16(p->params.data()), 0x1234);
  EXPECT_FALSE(parser.next().has_value());
  EXPECT_EQ(parser.checksumErrors(), 1u);
}

TEST(StsProtocol, SignMagnitude)
{
  EXPECT_EQ(decodeSigned(0x8000 | 300, 15), -300);
  EXPECT_EQ(decodeSigned(300, 15), 300);
  EXPECT_EQ(decodeSigned(0x400 | 250, 10), -250);
  EXPECT_EQ(encodeSigned(-250, 10), 0x400 | 250);
}

TEST(StsProtocol, AngleConversion)
{
  EXPECT_EQ(radiansToSteps(0.0, 1, 0.0), 2048);
  EXPECT_EQ(radiansToSteps(M_PI / 2, 1, 0.0), 3072);
  EXPECT_EQ(radiansToSteps(M_PI / 2, -1, 0.0), 1024);
  EXPECT_EQ(radiansToSteps(0.3, 1, 0.3), 2048);
  EXPECT_EQ(radiansToSteps(10.0, 1, 0.0), 4095);   // clamped
  for (int steps : {0, 777, 2048, 3333, 4095}) {
    for (int dir : {-1, 1}) {
      EXPECT_EQ(radiansToSteps(stepsToRadians(steps, dir, 0.12), dir, 0.12), steps);
    }
  }
  EXPECT_NEAR(stepsPerSecondToRadians(4096, 1), 2 * M_PI, 1e-12);
}
