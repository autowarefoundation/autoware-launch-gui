"use client";

import React from "react";
import dynamic from "next/dynamic";
import { invoke } from "@tauri-apps/api/core";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  autowareProcessesAtom,
  cpusUsageAtom,
  cpuUsageAtom,
  memoryUsageAtom,
  memoryUsagePercentageAtom,
  tabAtom,
  topProcessesAtom,
} from "@/app/jotai/atoms";

const LaunchMainEntry = dynamic(
  () => import("@/components/tabComponents/LaunchMainEntry"),
  {
    ssr: false,
  }
);
const RosbagPlayer = dynamic(
  () => import("@/components/tabComponents/RosbagPlayer"),
  {
    ssr: false,
  }
);

const TopicsBagRecord = dynamic(
  () => import("@/components/tabComponents/TopicsBagRecord"),
  {
    ssr: false,
  }
);

const TopicPublish = dynamic(
  () => import("@/components/tabComponents/TopicPublish"),
  {
    ssr: false,
  }
);

export const ServiceCall = dynamic(
  () => import("@/components/tabComponents/ServiceCall"),
  {
    ssr: false,
  }
);

export const Settings = dynamic(
  () => import("@/components/tabComponents/Settings"),
  {
    ssr: false,
  }
);

export default function App() {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );

  const [_cpuUsage, setCpuUsage] = useAtom(cpuUsageAtom);
  const [_memoryUsage, setMemoryUsage] = useAtom(memoryUsageAtom);
  const [_memoryUsagePercentage, setMemoryUsagePercentage] = useAtom(
    memoryUsagePercentageAtom
  );
  const [_cpusUsage, setCpusUsage] = useAtom(cpusUsageAtom);
  const [_topProcesses, setTopProcesses] = useAtom(topProcessesAtom);

  const [autowareProcessesNames, setAutowareProcessesNames] = useAtom(
    autowareProcessesAtom
  );

  const updateUsage = async () => {
    const systemInfo = (await invoke("get_system_info", {
      autowarePath: autowareFolderPath,
    })) as {
      cpu_usage: number;
      memory_used_percentage: number;
      memory_usage: string;
      cpus_usage: number[];
      top_processes: { name: string; cpu_usage: number }[];
      autoware_processes: string[];
    };
    setAutowareProcessesNames(systemInfo.autoware_processes);
    setCpuUsage(parseFloat(systemInfo.cpu_usage.toFixed(2)));
    // split the memory usag`e string to add a space between the number and GB (e.g. 3.5GB -> 3.5 GB)
    // then rejoins the string with the space added for example 1 Gb / 4 GB
    const memoryUsage = systemInfo.memory_usage.split(/([0-9.]+)/);
    setMemoryUsage(memoryUsage.join(" "));
    setMemoryUsagePercentage(
      parseFloat(systemInfo.memory_used_percentage.toFixed(2))
    );
    setCpusUsage(systemInfo.cpus_usage.map((u) => parseFloat(u.toFixed(2))));
    setTopProcesses(
      systemInfo.top_processes.map((p) => {
        return {
          name: p.name,
          cpu_usage: parseFloat(p.cpu_usage.toFixed(2)),
        };
      })
    );
  };

  React.useEffect(() => {
    updateUsage();
    const interval = setInterval(() => {
      updateUsage();
    }, 1000);
    return () => clearInterval(interval);
  }, [autowareFolderPath]);
  const [tab] = useAtom(tabAtom);

  return (
    <div className="relative flex h-full w-full select-none justify-between p-4">
      {tab === "launch" ? <LaunchMainEntry /> : null}
      {tab === "rosbag" ? <RosbagPlayer /> : null}
      {tab === "topics" ? <TopicsBagRecord /> : null}
      {tab === "publish" ? <TopicPublish /> : null}
      {tab === "service" ? <ServiceCall /> : null}
      {tab === "settings" ? <Settings /> : null}
    </div>
  );
}
