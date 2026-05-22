#include "signal_processor.hpp"
using namespace std;

SignalProcessor::SignalProcessor(int guard_cells, int reference_cells, float threshold_factor)
    : guard_(guard_cells)
    , ref_(reference_cells)
    , threshold_factor_(threshold_factor)
{
    result_buffer_.reserve(32);  // radar málokdy detekuje víc než 32 cílů najednou
}

vector<Detection> SignalProcessor::process(const Sweep& sweep) {
    result_buffer_.clear();  // clear() nezahazuje alokovanou paměť — to je ten trik

    int half_window = guard_ + ref_;

    for (int i = half_window; i < RANGE_BINS - half_window; ++i) {
        // TODO (Fáze 3): spočítej průměr referenčních buněk kolem i
        // (přeskoč guard_ buněk na každé straně)
        // práh = průměr * threshold_factor_
        // pokud sweep[i] > práh → push_back Detection{...} do result_buffer_

        float sum = 0;
        for (int j = i - half_window; j < i - guard_; ++j)
            sum += sweep[j];
        for (int j = i + guard_ + 1; j <= i + half_window; ++j)
            sum += sweep[j];

        float threshold = (sum / (2 * ref_)) * threshold_factor_;
        if (sweep[i] > threshold)
            result_buffer_.push_back({static_cast<uint16_t>(i), sweep[i]});
    }

    return result_buffer_;
}
