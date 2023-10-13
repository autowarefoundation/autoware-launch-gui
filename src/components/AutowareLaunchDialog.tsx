"use client";

import { memo, useCallback, useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { useAtom } from "jotai";

import {
  Dialog,
  DialogContent,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import {
  autowareFolderPathAtom,
  installedPackagesAtom,
  launchLogsComponentAtom,
  launchLogsDebugAtom,
  launchLogsErrorAtom,
  launchLogsInfoAtom,
  launchLogsWarnAtom,
  pidsLengthAtom,
} from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from "./ui/select";
import { Tabs, TabsList, TabsTrigger } from "./ui/tabs";

interface AutowareLaunchDialog extends React.HTMLAttributes<HTMLDivElement> {}

const tabTitles = ["INFO", "WARN", "ERROR", "DEBUG", "COMPONENT"];

export function AutowareLaunchDialog(props: AutowareLaunchDialog) {
  const logDivRef = useRef<HTMLDivElement>(null);
  const [pidslen] = useAtom(pidsLengthAtom);
  const [launchLogsInfo, setLaunchLogsInfo] = useAtom(launchLogsInfoAtom);
  const [launchLogsWarn, setLaunchLogsWarn] = useAtom(launchLogsWarnAtom);
  const [launchLogsError, setLaunchLogsError] = useAtom(launchLogsErrorAtom);
  const [launchLogsDebug, setLaunchLogsDebug] = useAtom(launchLogsDebugAtom);
  const [launchLogsComponent, setLaunchLogsComponent] = useAtom(
    launchLogsComponentAtom
  );
  const [autowareFolderPath, _setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );
  const [packageList, setPackageList] = useAtom(installedPackagesAtom);
  const [selectedPackage, setSelectedPackage] = useState("");

  const handlePackageSelect = useCallback((pkg: string) => {
    setSelectedPackage(pkg);
  }, []);

  useEffect(() => {
    async function init() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      // list out the folder names inside of the install folder inside of the autoware folder
      const folders: string[] = await invoke("autoware_installed_packages", {
        payload: {
          autowarePath: autowareFolderPath,
        },
      });
      if (packageList.length !== folders.length) {
        setPackageList(folders);
      }

      const unlistenLaunchLogs = await appWindow.listen(
        "autoware-output",
        (data) => {
          const logs = data.payload as string;

          // add logs to the launchLogs state and scroll to the bottom of the textarea, but if launchlogs is bigger than 100, remove the first 50, so it doesn't get too big and we stay at 50 logs
          // if logs includes "ERROR" or "WARN" or "DEBUG" or "INFO" add it to the corresponding state while taking out the first 50 if it's bigger than 100
          if (logs.includes("ERROR")) {
            setLaunchLogsError((prev) => {
              if (prev.length > 100) {
                return [...prev.slice(50), logs];
              } else {
                return [...prev, logs];
              }
            });
          } else if (logs.includes("WARN")) {
            setLaunchLogsWarn((prev) => {
              if (prev.length > 100) {
                return [...prev.slice(50), logs];
              } else {
                return [...prev, logs];
              }
            });
          } else if (logs.includes("DEBUG")) {
            setLaunchLogsDebug((prev) => {
              if (prev.length > 100) {
                return [...prev.slice(50), logs];
              } else {
                return [...prev, logs];
              }
            });
          } else if (logs.includes("INFO")) {
            setLaunchLogsInfo((prev) => {
              if (prev.length > 100) {
                return [...prev.slice(50), logs];
              } else {
                return [...prev, logs];
              }
            });
          }

          // if logs includes one of the package names, add it to the launchLogsComponent state while taking out the first 50 if it's bigger than 100
          if (folders.some((name) => logs.includes(name))) {
            // we add them according to their names, so we can easily see which package is causing the error
            // launchcomponentlogs is an array of objects, each object has a name and an array of logs
            setLaunchLogsComponent((prev) => {
              const newLogs = [...prev];
              const index = newLogs.findIndex((log) => logs.includes(log.name));
              if (index !== -1) {
                if (newLogs[index].logs.length > 100) {
                  newLogs[index].logs = [
                    ...newLogs[index].logs.slice(50),
                    logs,
                  ];
                } else {
                  newLogs[index].logs = [...newLogs[index].logs, logs];
                }
              } else {
                newLogs.push({
                  name: folders.find((name) => logs.includes(name)) ?? "",
                  logs: [logs],
                });
              }
              return newLogs;
            });
          }
          logDivRef.current?.scrollTo(0, logDivRef.current?.scrollHeight);
        }
      );

      return () => {
        unlistenLaunchLogs();
      };
    }
    init();
  }, []);

  const triggerRef = useRef<HTMLButtonElement>(null);
  const [currentTab, setCurrentTab] = useState(tabTitles[0].toLowerCase());

  // trigger the dialog to open when pidslen is bigger than 0 and automatically close when pidslen is > 35
  useEffect(() => {
    if (pidslen > 0) {
      triggerRef.current?.click();
    } else if (pidslen > 35) {
      triggerRef.current?.click();
    }
  }, [pidslen]);

  return (
    <Dialog>
      <DialogTrigger>
        <Button
          variant="default"
          ref={triggerRef}
          className={`flex w-fit items-center justify-center gap-2 ${
            pidslen === 0 ? "hidden" : ""
          }`}
          disabled={pidslen === 0}
        >
          <span>Launch Logs</span>
          <span className="text-xs opacity-50">({pidslen})</span>
        </Button>
      </DialogTrigger>
      <DialogContent className="flex h-[720px] max-w-[600px] flex-col items-center">
        <DialogTitle>Autoware Launch Logs</DialogTitle>
        <div
          ref={logDivRef}
          className="flex h-full w-full flex-col overflow-y-auto rounded-lg"
        >
          <Tabs
            defaultValue="info"
            className="flex w-full flex-col gap-2"
            onValueChange={(value) => {
              setCurrentTab(value);
            }}
          >
            <TabsList className="sticky top-0">
              {tabTitles.map((title) => (
                <TabsTrigger key={title} value={title.toLowerCase()}>
                  {title}
                </TabsTrigger>
              ))}
            </TabsList>
            {currentTab === "component" && (
              <div className="sticky left-0 top-10 z-50 flex items-center bg-primary-foreground p-2">
                <PackageDropdown
                  packages={packageList}
                  onPackageSelect={handlePackageSelect}
                />
              </div>
            )}
            <div className=" flex flex-col gap-4 overflow-y-auto break-words p-4">
              {currentTab === "info" &&
                launchLogsInfo.map((log, index) => (
                  <div key={index} className="">
                    {log}
                  </div>
                ))}
              {currentTab === "warn" &&
                launchLogsWarn.map((log, index) => (
                  <div key={index} className="text-yellow-500">
                    {log}
                  </div>
                ))}
              {currentTab === "error" &&
                launchLogsError.map((log, index) => (
                  <div key={index} className="text-red-500">
                    {log}
                  </div>
                ))}
              {currentTab === "debug" &&
                launchLogsDebug.map((log, index) => (
                  <div key={index} className="text-emerald-500">
                    {log}
                  </div>
                ))}
              {currentTab === "component" && (
                <div className="flex flex-col overflow-y-auto break-words p-4">
                  {/* Display logs for the selected package */}
                  {launchLogsComponent
                    .filter((log) => log.name === selectedPackage)
                    .map((logObj, index) => (
                      <div key={index} className="">
                        {logObj.logs.join("\n")}
                      </div>
                    ))}
                </div>
              )}
            </div>
          </Tabs>
        </div>
      </DialogContent>
    </Dialog>
  );
}

const PackageDropdown = memo(function PackageDropdown({
  packages,
  onPackageSelect,
}: {
  packages: string[];
  onPackageSelect: (selectedPackage: string) => void;
}) {
  const [selectedPackage, setSelectedPackage] = useState("");

  const handlePackageSelect = useCallback(
    (value: string) => {
      const pkg = value;
      console.log(pkg);
      setSelectedPackage(pkg);
      onPackageSelect(pkg);
    },
    [onPackageSelect]
  );

  return (
    <Select onValueChange={handlePackageSelect}>
      <SelectTrigger className="w-fit">
        <SelectValue placeholder={selectedPackage || "Select package..."} />
      </SelectTrigger>
      <SelectContent>
        <SelectGroup
          style={{
            maxHeight: "12rem",
            overflowY: "auto",
            position: "relative",
          }}
        >
          <SelectLabel>Packages</SelectLabel>
          {packages
            .sort((a, b) => a.localeCompare(b))
            .map((pkg: string) => (
              <SelectItem key={pkg} value={pkg}>
                {pkg}
              </SelectItem>
            ))}
        </SelectGroup>
      </SelectContent>
    </Select>
  );
});
