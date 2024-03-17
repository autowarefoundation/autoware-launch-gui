"use client";

import { useCallback, useEffect, useState } from "react";
import { BarChart } from "@tremor/react";
import { useAtom } from "jotai";
import { Loader2 } from "lucide-react";

import { cn } from "@/lib/utils";
import {
  cpusUsageAtom,
  cpuUsageAtom,
  memoryUsageAtom,
  memoryUsagePercentageAtom,
  topProcessesAtom,
} from "@/app/jotai/atoms";

import { Progress } from "../ui/progress";
import { isJSONParsable } from "../WebSocket";
import Launch from "./Launch";

const isWindow = typeof window !== "undefined";

const LaunchMainEntry = () => {
  const [cpuUsage, setCpuUsage] = useAtom(cpuUsageAtom);
  const [memoryUsage, setMemoryUsage] = useAtom(memoryUsageAtom);
  const [memoryUsagePercentage, setMemoryUsagePercentage] = useAtom(
    memoryUsagePercentageAtom
  );
  const [cpusUsage, setCpusUsage] = useAtom(cpusUsageAtom);
  const [topProcesses, setTopProcesses] = useAtom(topProcessesAtom);
  const [socket, setSocket] = useState<WebSocket | null>(null);

  useEffect(() => {
    let ws: WebSocket;
    if (isWindow) {
      ws = new WebSocket("ws://localhost:42068/ws");
      ws.onopen = () => {
        console.log("connected to app/browser syncing websocket");

        // @ts-ignore
        if (window.__TAURI__) {
          ws.send("Tauri-systemInfo");
        } else {
          ws.send("Browser-systemInfo");
        }
      };
      ws.onclose = () => {
        console.log("connection to app/browser syncing websocket closed");
      };
      ws.onerror = (e) => {
        console.log("error from websocket", e);
      };

      setSocket(ws);
    }

    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, []);

  const handleTauriMessages = () => {
    // @ts-ignore
    if (isWindow && window.__TAURI__) {
      if (socket) {
        socket.send(
          JSON.stringify({
            typeOfMessage: "getSystemInfo",
            message: {
              cpuUsage,
              memoryUsage,
              cpusUsage,
              topProcesses,
              memoryUsagePercentage,
            },
          })
        );
      }
      // @ts-ignore
    } else if (isWindow && !window.__TAURI__) {
      if (socket) {
        socket.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (isJSONParsable(data.message)) {
            const parsedData = JSON.parse(data.message) as {
              typeOfMessage: string;
              message: {
                cpuUsage: number;
                memoryUsage: string;
                cpusUsage: number[];
                topProcesses: { name: string; cpu_usage: number }[];
                memoryUsagePercentage: number;
              };
            };

            if (parsedData.typeOfMessage === "getSystemInfo") {
              setCpuUsage(parsedData.message.cpuUsage);
              setMemoryUsage(parsedData.message.memoryUsage);
              setCpusUsage(parsedData.message.cpusUsage);
              setTopProcesses(parsedData.message.topProcesses);
              setMemoryUsagePercentage(
                parsedData.message.memoryUsagePercentage
              );
            }
          }
        };
      }
    }
  };

  useEffect(() => {
    handleTauriMessages();
  }, [
    handleTauriMessages,
    cpuUsage,
    memoryUsage,
    cpusUsage,
    topProcesses,
    memoryUsagePercentage,
  ]);

  return (
    <div className="flex h-full w-full flex-col gap-4">
      <Launch />
      {/* @ts-ignore */}

      <div className="flex h-full w-full flex-row gap-4 font-mono">
        {cpusUsage.every((u) => u === 0) &&
        topProcesses[0].name === "process1" ? (
          <div
            role="status"
            // put it in the middle of the parent div
            className="flex h-full w-full flex-col items-center justify-center"
          >
            <Loader2 className="mr-2 inline h-24 w-24 animate-spin text-blue-500 transition-all" />
            <span className="sr-only">Loading...</span>
          </div>
        ) : (
          <div className="flex h-full w-full gap-2">
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
                  "cpu percentage": percentage,
                  name: `${index}`,
                }))}
                className="font-mono text-xs"
                index="name"
                categories={["cpu percentage"]}
                colors={["blue"]}
                showGridLines={false}
                showTooltip={false}
                // showLegend={false}
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
          </div>
        )}
      </div>
    </div>
  );
};

export default LaunchMainEntry;
