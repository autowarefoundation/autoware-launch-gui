// @ts-nocheck
"use client";

import React from "react";
import { invoke } from "@tauri-apps/api/core";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  autowareProcessesAtom,
  bagFileAtom,
  bagFileInfoAtom,
  cpusUsageAtom,
  cpuUsageAtom,
  installedPackagesAtom,
  isBagPlayingAtom,
  lastSavedLoadedProfileJSONPathsAtom,
  memoryUsageAtom,
  memoryUsagePercentageAtom,
  parsedLaunchFilePathAtom,
  parsedLaunchFilesAtom,
  socketAtom,
  tabAtom,
  topProcessesAtom,
  userEditedArgsAtom,
  userEditedBagPlayFlagsAtom,
  userEditedBagRecordFlagsAtom,
  userEditedServiceCallFlagsAtom,
  userEditedTopicPubFlagsAtom,
} from "@/app/jotai/atoms";

import {
  isJSONParsable,
  isWindow,
  LaunchMainEntry,
  RosbagPlayer,
  ServiceCall,
  Settings,
  TopicPublish,
  TopicsBagRecord,
} from "./page";

export default function App() {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );

  const [socket, setSocket] = useAtom(socketAtom);

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
    // @ts-ignore
    if (isWindow && window.__TAURI__) {
      updateUsage();
      const interval = setInterval(() => {
        updateUsage();
      }, 1000);
      return () => clearInterval(interval);
    }
  }, [autowareFolderPath]);
  const [tab] = useAtom(tabAtom);

  React.useEffect(() => {
    let ws: WebSocket;
    if (isWindow) {
      ws = new WebSocket("ws://localhost:42068/ws");
      ws.onopen = () => {
        console.log("connected to app/browser syncing websocket");

        if (window.__TAURI__) {
          ws.send("Tauri");
        } else {
          ws.send("Browser");
          if (!autowareFolderPath) {
            ws.send(JSON.stringify({ autowareFolderPath: autowareFolderPath }));
          }
        }
      };
      ws.onclose = () => {
        console.log("connection to app/browser syncing websocket closed");
      };
      ws.onerror = (e) => {
        console.log("error from websocket", e);
      };

      ws.onmessage = (e) => {
        onSocketMessage(e);
      };
      setSocket(ws);
    }

    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [autowareFolderPath, isWindow]);
  /**
   * localStorage items that are needed to be sent to the localhost server to sync the browser with the desktop app
   *
   * 1. autowareFolderPathLaunchGUI
   * 2. bagFileInfoLaunchGUI
   * 3. bagFileLaunchGUI
   * 4. installedPackagesLaunchGUI
   * 5. isBagPlayingLaunchGUI
   * 6. lastSavedLoadedProfileJSONPathsLaunchGUI
   * 7. parsedLaunchFilePathLaunchGUI
   * 8. parsedLaunchFilesLaunchGUI
   * 9. userEditedArgsLaunchGUI
   * 10. userEditedBagPlayFlagsLaunchGUI
   * 11. userEditedBagRecordFlagsLaunchGUI
   * 12. userEditedServiceCallFlagsLaunchGUI
   * 13. userEditedTopicPubFlagsLaunchGUI
   *
   * the localhost server is only available when the built desktop app is running and the server is started on port 42069
   * and as such we should be able to send the data to the server at the /api/sync endpoint
   * as soon as we confirm that at least one of the above items is not null which means that the user has interacted with the app
   * and that the frontend has some data that needs to be synced with the localhost server
   */
  const [bagFileInfo] = useAtom(bagFileInfoAtom);
  const [bagFile] = useAtom(bagFileAtom);
  const [installedPackages] = useAtom(installedPackagesAtom);
  const [isBagPlaying] = useAtom(isBagPlayingAtom);
  const [lastSavedLoadedProfileJSONPaths] = useAtom(
    lastSavedLoadedProfileJSONPathsAtom
  );
  const [parsedLaunchFilePath] = useAtom(parsedLaunchFilePathAtom);
  const [parsedLaunchFiles] = useAtom(parsedLaunchFilesAtom);
  const [userEditedArgs] = useAtom(userEditedArgsAtom);
  const [userEditedBagPlayFlags] = useAtom(userEditedBagPlayFlagsAtom);
  const [userEditedBagRecordFlags] = useAtom(userEditedBagRecordFlagsAtom);
  const [userEditedServiceCallFlags] = useAtom(userEditedServiceCallFlagsAtom);
  const [userEditedTopicPublishFlags] = useAtom(userEditedTopicPubFlagsAtom);

  const onSocketMessage = (e: MessageEvent) => {
    if (isJSONParsable(e.data)) {
      //  we then check if the parsed JSON object has a message property
      const parsedData: {
        name: "Tauri" | "Browser";
        message: string;
      } = JSON.parse(e.data);

      if (parsedData.message) {
        //  if it does we then check if the name is "Tauri" then it has been sent to our localhost and if the message is json parsable
        if (
          parsedData.name === "Tauri" &&
          isJSONParsable(parsedData.message) &&
          !window.__TAURI__
        ) {
          const parsedMessage: {
            autowareFolderPath: string | null;
          } = JSON.parse(parsedData.message);
          console.log("parsedMessageInBrowser", parsedMessage);
          if (parsedMessage.autowareFolderPath)
            setAutowareFolderPath(parsedMessage.autowareFolderPath);
          else {
            // we ask for one from the user
            ws.send(JSON.stringify({ autowareFolderPath: autowareFolderPath }));
          }
        }
        if (
          parsedData.name === "Browser" &&
          isJSONParsable(parsedData.message) &&
          window.__TAURI__
        ) {
          const parsedMessage: {
            autowareFolderPath: string | null;
          } = JSON.parse(parsedData.message);
          console.log(parsedMessage.autowareFolderPath, autowareFolderPath);
          if (parsedMessage.autowareFolderPath === null) {
            ws.send(JSON.stringify({ autowareFolderPath: autowareFolderPath }));
          }
        }
      }
    }
  };

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
