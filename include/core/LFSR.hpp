/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

template <typename T, T POLY, T DEFAULT = POLY>
struct LFSR {
public:
    using Type = T;
    static constexpr Type
    POLYNOMIAL()
    {
        return POLY;
    }

    LFSR() : _x(DEFAULT) {}

    LFSR(
        Type x
    ) : _x(x) {}

    inline Type
    operator()()
    {
        return update();
    }

    inline Type
    operator()(
        Type x
    )
    {
        _x = x;

        return update();
    }

    inline Type
    current()
    {
        return _x;
    }

    inline Type
    next()
    {
        return update();
    }

private:
    Type _x;
    inline Type
    update()
    {
        if (DEFAULT != 0) {
            if (_x == 0) {
                _x = DEFAULT;
            }
        }

        if (_x & 1) {
            _x >>= 1;
        } else {
            _x >>= 1;
            _x  ^= POLY;
        }

        return _x;
    }
};
