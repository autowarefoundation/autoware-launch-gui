"use client";

import { BarChart } from "@tremor/react";
import { useAtom } from "jotai";

import { cn } from "@/lib/utils";
import {
  cpusUsageAtom,
  cpuUsageAtom,
  memoryUsageAtom,
  memoryUsagePercentageAtom,
  topProcessesAtom,
} from "@/app/jotai/atoms";

import { Progress } from "../ui/progress";
import Launch from "./Launch";

// const listOfQuickstartItems = [
//   "Map",
//   "Sensing",
//   "Localization",
//   "Detection",
//   "Mission Planning",
//   "Motion Planning",
// ];

// function rowOfQuickstartItems(item: string) {
//   const [value, setValue] = React.useState("");
//   const inputRef = React.useRef<HTMLInputElement>(null);

//   return (
//     <div key={item} className="flex flex-row justify-between gap-4">
//       <Button className="w-fit" variant={"default"}>
//         {item}
//       </Button>
//       <Input
//         placeholder={"Enter value"}
//         ref={inputRef}
//         value={value}
//         onChange={(e) => setValue(e.target.value)}
//       />
//       <Button variant={"default"}>Ref</Button>
//     </div>
//   );
// }

const CPUMemUsage = () => {
  const [cpuUsage, setCpuUsage] = useAtom(cpuUsageAtom);
  const [memoryUsage, setMemoryUsage] = useAtom(memoryUsageAtom);
  const [memoryUsagePercentage, setMemoryUsagePercentage] = useAtom(
    memoryUsagePercentageAtom
  );
  const [cpusUsage, setCpusUsage] = useAtom(cpusUsageAtom);
  const [topProcesses, setTopProcesses] = useAtom(topProcessesAtom);

  return (
    <div className="flex h-full flex-1 flex-col justify-between gap-4">
      <Launch />

      <div className="relative flex flex-row justify-between gap-4 font-mono">
        {cpusUsage.every((u) => u === 0) &&
        topProcesses[0].name === "process1" ? (
          <div
            role="status"
            // put it in the middle of the parent div
            className="absolute top-0"
          >
            <svg
              aria-hidden="true"
              className="mr-2 inline h-10 w-10 animate-spin fill-blue-600 text-gray-200 dark:text-gray-600"
              viewBox="0 0 100 101"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M100 50.5908C100 78.2051 77.6142 100.591 50 100.591C22.3858 100.591 0 78.2051 0 50.5908C0 22.9766 22.3858 0.59082 50 0.59082C77.6142 0.59082 100 22.9766 100 50.5908ZM9.08144 50.5908C9.08144 73.1895 27.4013 91.5094 50 91.5094C72.5987 91.5094 90.9186 73.1895 90.9186 50.5908C90.9186 27.9921 72.5987 9.67226 50 9.67226C27.4013 9.67226 9.08144 27.9921 9.08144 50.5908Z"
                fill="currentColor"
              />
              <path
                d="M93.9676 39.0409C96.393 38.4038 97.8624 35.9116 97.0079 33.5539C95.2932 28.8227 92.871 24.3692 89.8167 20.348C85.8452 15.1192 80.8826 10.7238 75.2124 7.41289C69.5422 4.10194 63.2754 1.94025 56.7698 1.05124C51.7666 0.367541 46.6976 0.446843 41.7345 1.27873C39.2613 1.69328 37.813 4.19778 38.4501 6.62326C39.0873 9.04874 41.5694 10.4717 44.0505 10.1071C47.8511 9.54855 51.7191 9.52689 55.5402 10.0491C60.8642 10.7766 65.9928 12.5457 70.6331 15.2552C75.2735 17.9648 79.3347 21.5619 82.5849 25.841C84.9175 28.9121 86.7997 32.2913 88.1811 35.8758C89.083 38.2158 91.5421 39.6781 93.9676 39.0409Z"
                fill="currentFill"
              />
            </svg>
            <span className="sr-only">Loading...</span>
          </div>
        ) : (
          <>
            <div
              className={cn(
                "w-2/3 flex-1 rounded-lg border p-2",
                cpusUsage.every((u) => u === 0) && "opacity-0",
                "transition-opacity duration-200"
              )}
            >
              <span>General CPU usage: {cpuUsage}%</span>
              <BarChart
                // yAxisWidth={250}
                data={cpusUsage.map((percentage, index) => ({
                  id: index.toString(),
                  value: percentage,
                  name: `cpu${index}`,
                }))}
                index="name"
                categories={["value"]}
                colors={["blue"]}
                showGridLines={false}
                showTooltip={false}
                showLegend={false}
                valueFormatter={(value) => `${value}%`}
              />
            </div>

            <div
              className={cn(
                "flex w-1/3 flex-col gap-4 rounded-lg border p-2",
                topProcesses[0].name === "process1" && "opacity-0",
                "transition-opacity duration-200"
              )}
            >
              <div className="flex flex-col gap-2">
                <span>Top processes:</span>
                {topProcesses.map((p, idx) => (
                  <span className="pl-4" key={`${p.name},${idx}`}>
                    <span className="font-semibold">{p.name}</span>:{" "}
                    {p.cpu_usage}% cpu
                  </span>
                ))}
              </div>
              <span>Memory used: {memoryUsage}</span>
              <Progress
                value={memoryUsagePercentage}
                className="h-4 w-[90%] self-center"
              />
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default CPUMemUsage;
