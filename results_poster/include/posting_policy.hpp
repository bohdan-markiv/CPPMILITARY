#pragma once
#include <optional>

namespace poster
{

    enum class Decision
    {
        Success,
        Fatal,
        Retry,
        GaveUp
    };

    // std::nullopt == no response at all (timeout / connection failure)
    using MaybeStatus = std::optional<int>;

    constexpr int kMaxAttempts = 5;

    inline Decision classify(MaybeStatus status)
    {
        if (!status)
        {
            return Decision::Retry;
        }
        else if (*status >= 200 && *status < 300)
        {
            return Decision::Success;
        }
        else if (*status >= 400 && *status < 500)
        {
            return Decision::Fatal;
        }
        else
        {
            return Decision::Retry;
        }
    }

    inline Decision decide(MaybeStatus status, int attempt_number)
    {
        Decision decision = classify(status);
        if (decision == Decision::Retry)
        {
            if (attempt_number >= kMaxAttempts)
            {
                return Decision::GaveUp;
            }
            else
            {
                return Decision::Retry;
            }
        }
        else
        {
            return decision;
        }
    }

}