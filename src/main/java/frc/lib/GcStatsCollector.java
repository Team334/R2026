package frc.lib;

import dev.doglog.DogLog;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class GcStatsCollector {
  private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
  private final long[] lastTimes = new long[gcBeans.size()];
  private final long[] lastCounts = new long[gcBeans.size()];

  public void update() {
    long accumTime = 0;
    long accumCounts = 0;
    for (int i = 0; i < gcBeans.size(); i++) {
      long gcTime = gcBeans.get(i).getCollectionTime();
      long gcCount = gcBeans.get(i).getCollectionCount();
      accumTime += gcTime - lastTimes[i];
      accumCounts += gcCount - lastCounts[i];

      lastTimes[i] = gcTime;
      lastCounts[i] = gcCount;
    }

    DogLog.log("GC/GCTimeMS", (double) accumTime);
    DogLog.log("GC/GCCounts", (double) accumCounts);
  }
}
