"use client";

import React from "react";
import dynamic from "next/dynamic";
import { invoke } from "@tauri-apps/api/tauri";
import { useAtom } from "jotai";

// import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  autowareFolderPathAtom,
  autowareProcessesAtom,
  cpusUsageAtom,
  cpuUsageAtom,
  memoryUsageAtom,
  memoryUsagePercentageAtom,
  topProcessesAtom,
} from "@/app/jotai/atoms";

const CPUMemUsage = dynamic(
  () => import("@/components/tabComponents/CPUMemUsage"),
  {
    ssr: false,
  }
);

// const tabTitles = ["Quick Start", "Setup"];
// type TabContentComponent = React.ComponentType<any> | string;

// const tabContentComponentsMapping: Record<string, TabContentComponent> = {
//   "Quick Start": Quickstart,
//   Setup: Setup,
//   Map: Map,
// };

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
      payload: {
        autowarePath: autowareFolderPath,
      },
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

  return (
    <div className="relative flex h-full w-full select-none justify-between p-4">
      <CPUMemUsage />
    </div>
  );
}

// const TabContentComponentsMappingComponent = () => {
//   const [autowareFolderPath, setAutowareFolderPath] = useAtom(
//     autowareFolderPathAtom
//   );
//   const [currentTab, setCurrentTab] = React.useState(
//     tabTitles[0].toLowerCase()
//   );
//   return (
//     <Tabs
//       defaultValue={tabTitles[0].toLowerCase()}
//       className="w-full"
//       onValueChange={(value) => {
//         setCurrentTab(value);
//       }}
//     >
//       {/* last item pushed away from the rest css */}
//       <TabsList>
//         {tabTitles.map((title) => (
//           <TabsTrigger
//             disabled={title !== "Quick Start" && !autowareFolderPath}
//             key={title}
//             value={title.toLowerCase()}
//           >
//             {title}
//           </TabsTrigger>
//         ))}
//       </TabsList>

//       <div className="p-4 pb-12">
//         {tabTitles.map((title) => (
//           <TabsContent key={title} value={title.toLowerCase()}>
//             {React.createElement(tabContentComponentsMapping[title])}
//           </TabsContent>
//         ))}
//       </div>
//     </Tabs>
//   );
// };
