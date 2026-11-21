// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "common/input.h"
#include "common/logging.h"
#include "input_common/helpers/joycon_protocol/ringcon.h"

#include <chrono>
#include <thread>

namespace InputCommon::Joycon {

RingConProtocol::RingConProtocol(std::shared_ptr<JoyconHandle> handle)
    : JoyconCommonProtocol(std::move(handle)) {}

Common::Input::DriverResult RingConProtocol::EnableRingCon() {
    LOG_DEBUG(Input, "Enable Ringcon");
    ScopedSetBlocking sb(this);
    Common::Input::DriverResult result{Common::Input::DriverResult::Success};

    const auto send_step = [&](SubCommand command, std::span<const u8> payload,
                               const char* name, std::chrono::milliseconds delay) {
        // Bluetooth HID can lose a subcommand reply while the Joy-Con changes
        // report modes.  The reference implementation retries each step;
        // a single missed reply must not abort an otherwise valid init.
        auto step_result = Common::Input::DriverResult::Timeout;
        for (int attempt = 0; attempt < 8; ++attempt) {
            step_result = SendSubCommand(command, payload);
            if (step_result == Common::Input::DriverResult::Success) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(64));
        }
        if (step_result != Common::Input::DriverResult::Success) {
            LOG_ERROR(Input, "Ringcon init step {} failed with error {}", name, step_result);
        }
        std::this_thread::sleep_for(delay);
        return step_result;
    };

    // Clear any stale MCU state.  A second suspend is intentionally best
    // effort: some firmware versions acknowledge the first command and then
    // stop replying while entering suspend.
    const std::array<u8, 1> suspend{0};
    (void)SendSubCommand(SubCommand::SET_MCU_STATE, suspend);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    (void)SendSubCommand(SubCommand::SET_MCU_STATE, suspend);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Use the Ring-Con initialization order required by the Joy-Con MCU.
    if (result == Common::Input::DriverResult::Success) {
        const std::array<u8, 1> vibration{1};
        result = send_step(SubCommand::ENABLE_VIBRATION, vibration, "vibration",
                           std::chrono::milliseconds(100));
    }
    if (result == Common::Input::DriverResult::Success) {
        const std::array<u8, 1> imu{1};
        result = send_step(SubCommand::ENABLE_IMU, imu, "IMU", std::chrono::milliseconds(100));
    }
    if (result == Common::Input::DriverResult::Success) {
        const std::array<u8, 1> report_mode{static_cast<u8>(ReportMode::STANDARD_FULL_60HZ)};
        result = send_step(SubCommand::SET_REPORT_MODE, report_mode, "report mode",
                           std::chrono::milliseconds(100));
    }
    if (result == Common::Input::DriverResult::Success) {
        const std::array<u8, 1> standby{1};
        result = send_step(SubCommand::SET_MCU_STATE, standby, "MCU standby",
                           std::chrono::milliseconds(1500));
    }
    if (result == Common::Input::DriverResult::Success) {
        // MCU ConfigureMCU(SetMCUMode, MaybeRingcon).  This is the 0x21
        // configuration packet with payload [0x21, 0x00, 0x03].
        const MCUConfig config{
            .command = MCUCommand::ConfigureMCU,
            .sub_command = MCUSubCommand::SetMCUMode,
            .mode = MCUMode::Ringcon,
            .crc = {},
        };
        // The MCU may stop producing normal subcommand replies while it
        // changes mode.  Resend the mode request until the transition is
        // acknowledged, matching the reference HID implementation.
        result = Common::Input::DriverResult::Timeout;
        for (int attempt = 0; attempt < 16; ++attempt) {
            result = ConfigureMCU(config);
            if (result == Common::Input::DriverResult::Success) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(64));
        }
        if (result != Common::Input::DriverResult::Success) {
            LOG_ERROR(Input, "Ringcon init step MaybeRingcon failed with error {}", result);
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    if (result == Common::Input::DriverResult::Success) {
        // MCU ConfigureMCU(SetIRMode, IRSensorSleep).  The existing
        // MCUConfig type represents the mode-setting form, so build the
        // six-byte IR payload explicitly while retaining the same CRC rules.
        std::array<u8, sizeof(MCUConfig)> ir_config{};
        ir_config[0] = static_cast<u8>(MCUCommand::ConfigureMCU);
        ir_config[1] = static_cast<u8>(MCUSubCommand::SetDeviceMode);
        ir_config[2] = 1; // IRSensorSleep
        ir_config[3] = 0; // no_of_frags
        ir_config[4] = 0; // firmware major (low)
        ir_config[5] = 0; // firmware major (high)
        ir_config[6] = 0; // firmware minor (low)
        ir_config[7] = 0; // firmware minor (high)
        ir_config[37] = CalculateMCU_CRC8(ir_config.data() + 1, 36);
        result = send_step(SubCommand::SET_MCU_CONFIG, ir_config, "IR sleep",
                           std::chrono::milliseconds(1000));
    }

    return result;
}

Common::Input::DriverResult RingConProtocol::DisableRingCon() {
    LOG_DEBUG(Input, "Disable RingCon");
    ScopedSetBlocking sb(this);
    Common::Input::DriverResult result{Common::Input::DriverResult::Success};

    if (result == Common::Input::DriverResult::Success) {
        result = EnableMCU(false);
    }

    is_enabled = false;

    return result;
}

Common::Input::DriverResult RingConProtocol::StartRingconPolling() {
    LOG_DEBUG(Input, "Enable Ringcon");
    ScopedSetBlocking sb(this);
    Common::Input::DriverResult result{Common::Input::DriverResult::Success};
    bool is_connected = false;

    if (result == Common::Input::DriverResult::Success) {
        result = IsRingConnected(is_connected);
    }
    if (result == Common::Input::DriverResult::Success && is_connected) {
        LOG_INFO(Input, "Ringcon detected");
        // 0x40 [0x03] switches the IMU payload to MaybeRingcon mode.  Without
        // this, bytes 39-40 are ordinary Joy-Con IMU data rather than Ring-Con
        // strain values.
        const std::array<u8, 1> maybe_ringcon{3};
        result = SendSubCommand(SubCommand::ENABLE_IMU, maybe_ringcon);
    }
    if (result == Common::Input::DriverResult::Success && is_connected) {
        result = ConfigureRing();
    }
    if (result == Common::Input::DriverResult::Success) {
        is_enabled = true;
    }

    return result;
}

Common::Input::DriverResult RingConProtocol::IsRingConnected(bool& is_connected) {
    LOG_DEBUG(Input, "IsRingConnected");
    constexpr std::size_t max_tries = 42;
    SubCommandResponse output{};
    std::size_t tries = 0;
    is_connected = false;

    do {
        const auto result = SendSubCommand(SubCommand::GET_EXTERNAL_DEVICE_INFO, {}, output);

        if (result != Common::Input::DriverResult::Success &&
            result != Common::Input::DriverResult::Timeout) {
            return result;
        }

        if (tries++ >= max_tries) {
            return Common::Input::DriverResult::NoDeviceDetected;
        }
    } while (output.external_device_id != ExternalDeviceId::RingController);

    is_connected = true;
    return Common::Input::DriverResult::Success;
}

Common::Input::DriverResult RingConProtocol::ConfigureRing() {
    LOG_DEBUG(Input, "ConfigureRing");

    static constexpr std::array<u8, 38> ring_config{
        0x06, 0x03, 0x25, 0x06, 0x00, 0x00, 0x00, 0x00, 0xEC, 0x99, 0xAC, 0xE3, 0x1C,
        0x00, 0x00, 0x00, 0x69, 0x9B, 0x16, 0xF6, 0x5D, 0x56, 0x00, 0x00, 0x04, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x28, 0xA1, 0xE3, 0x1C, 0x00};

    const Common::Input::DriverResult result =
        SendSubCommand(SubCommand::SET_EXTERNAL_FORMAT_CONFIG, ring_config);

    if (result != Common::Input::DriverResult::Success) {
        return result;
    }

    static constexpr std::array<u8, 4> ringcon_data{0x04, 0x01, 0x01, 0x02};
    return SendSubCommand(SubCommand::ENABLE_EXTERNAL_POLLING, ringcon_data);
}

bool RingConProtocol::IsEnabled() const {
    return is_enabled;
}

} // namespace InputCommon::Joycon
