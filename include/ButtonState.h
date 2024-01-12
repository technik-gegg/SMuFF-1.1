/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

typedef enum _ButtonState {
    Open = 0,
    Closed,
    Pressed,
    Held,
    Released,
    Clicked,
    DoubleClicked,
    LongClicked,
    LeftTurn,
    RightTurn
} ButtonState;

#if !defined(USE_LEONERD_DISPLAY)

// Buttons
typedef enum _Buttons {
    NoButton = 0,
    WheelButton,
    MainButton,
    LeftButton,
    RightButton
} Buttons;
#endif
