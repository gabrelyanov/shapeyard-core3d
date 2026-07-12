#pragma once

#include <mutex>

// OCCT's STEP exchange stack uses process-global state. Keep every reader and
// writer lifecycle behind this shared lock, including object destruction.
inline std::mutex& Core3DSTEPExchangeMutex()
{
    static std::mutex aMutex;
    return aMutex;
}
