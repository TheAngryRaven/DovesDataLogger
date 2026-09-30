#include "sd_probe.h"

namespace sd_probe {

Verdict classify(const Attempt& a, int volumeFailuresBefore) {
  // Card layer first: no SPI-level answer, or no size, is a dead/absent
  // card whatever the earlier probes said. An erase is only ever offered
  // to a card that has answered consistently.
  if (!a.cardOk || !a.hasSectors) return Verdict::kDead;
  if (a.volumeOk) return Verdict::kMounted;

  const int failures = volumeFailuresBefore + 1;
  if (failures >= kVolumeFailuresToDeclare) return Verdict::kUnformatted;
  return Verdict::kRetryVolume;
}

}  // namespace sd_probe
