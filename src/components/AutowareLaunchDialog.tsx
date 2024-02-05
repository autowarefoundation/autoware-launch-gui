"use client";

import { memo, useCallback, useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
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
  launchLogsAllAtom,
  launchLogsComponentAtom,
  launchLogsDebugAtom,
  launchLogsErrorAtom,
  launchLogsInfoAtom,
  launchLogsWarnAtom,
  pidsLengthAtom,
} from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import { Input } from "./ui/input";
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

const tabTitles = ["ALL", "INFO", "WARN", "ERROR", "DEBUG", "COMPONENT"];

export function AutowareLaunchDialog(props: AutowareLaunchDialog) {
  const logDivRef = useRef<HTMLDivElement>(null);
  const [pidslen] = useAtom(pidsLengthAtom);
  const [launchLogsAll, setLaunchLogsAll] = useAtom(launchLogsAllAtom);
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
      // list out the folder names inside of the install folder inside of the autoware folder
      const folders: string[] = await invoke("autoware_installed_packages", {
        autowarePath: autowareFolderPath,
      });
      if (packageList.length !== folders.length) {
        setPackageList(folders);
      }

      const unlistenLaunchLogs = await listen("autoware-output", (data) => {
        const logs = data.payload as string;

        // Helper function to handle log updates using Sets
        const handleLogUpdate = (setLogState: any) => {
          setLogState((prevArray: string[]) => {
            // Deep clone the previous array
            const clonedPrevArray = JSON.parse(JSON.stringify(prevArray));

            const newSet = new Set(clonedPrevArray);
            newSet.add(logs);
            if (newSet.size > 100) {
              return [...newSet].slice(-50);
            }
            return [...newSet];
          });
        };

        // Check for "ERROR" logs
        if (logs.includes("ERROR")) {
          console.log("WE HAVE AN ERROR");
          handleLogUpdate(setLaunchLogsAll);
          handleLogUpdate(setLaunchLogsError);
        }

        // Check for "WARN" logs
        if (logs.includes("WARN")) {
          console.log("WE HAVE A WARNING");
          handleLogUpdate(setLaunchLogsAll);
          handleLogUpdate(setLaunchLogsWarn);
        }

        // Check for "DEBUG" logs
        if (logs.includes("DEBUG")) {
          console.log("WE HAVE A DEBUG LOG");
          handleLogUpdate(setLaunchLogsAll);
          handleLogUpdate(setLaunchLogsDebug);
        }

        // Check for "INFO" logs
        if (logs.includes("INFO")) {
          console.log("WE HAVE AN INFO LOG");
          handleLogUpdate(setLaunchLogsAll);
          handleLogUpdate(setLaunchLogsInfo);
        }

        // Always update the "ALL" logs
        handleLogUpdate(setLaunchLogsAll);

        // if logs includes one of the package names, add it to the launchLogsComponent state
        if (folders.some((name) => logs.includes(name))) {
          setLaunchLogsComponent((prev) => {
            const newLogs = [...prev];
            const index = newLogs.findIndex((log) => logs.includes(log.name));
            if (index !== -1) {
              const updatedLogs = new Set(newLogs[index].logs);
              updatedLogs.add(logs);
              if (updatedLogs.size > 100) {
                newLogs[index].logs = [...updatedLogs].slice(-50);
              } else {
                newLogs[index].logs = [...updatedLogs];
              }
            } else {
              newLogs.push({
                name: folders.find((name) => logs.includes(name)) ?? "",
                logs: [...new Set([logs])],
              });
            }
            return newLogs;
          });
        }
      });

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

  const [searchQuery, setSearchQuery] = useState("");
  function filterLogs(logs: string[]): string[] {
    if (!searchQuery) return logs;
    return logs.filter((log) =>
      log.toLowerCase().includes(searchQuery.toLowerCase())
    );
  }

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
          <span>Logs</span>
          <span className="text-xs opacity-50">({pidslen})</span>
        </Button>
      </DialogTrigger>
      <DialogContent className="flex h-[720px] max-w-[600px] flex-col items-center">
        <DialogTitle>Autoware Launch Logs</DialogTitle>
        <div
          ref={logDivRef}
          className="relative flex h-full w-full flex-col overflow-y-auto rounded-lg"
        >
          <Tabs
            defaultValue="all"
            className="flex w-full flex-col gap-2"
            onValueChange={(value) => {
              setCurrentTab(value);
            }}
          >
            <TabsList className="fixed top-4">
              {tabTitles.map((title) => (
                <TabsTrigger key={title} value={title.toLowerCase()}>
                  {title}
                </TabsTrigger>
              ))}
            </TabsList>
            {currentTab === "component" ? (
              <div className="fixed left-4 top-16 z-50 flex items-center bg-primary-foreground p-2">
                <PackageDropdown
                  packages={packageList}
                  onPackageSelect={handlePackageSelect}
                />
              </div>
            ) : (
              <Input
                type="search"
                placeholder="Filter out your logs..."
                className="fixed left-7 top-16 z-50 w-3/4 font-mono text-xs"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
              />
            )}
            <div className="fixed left-0 top-28 flex max-h-[calc(100%-8rem)] w-full flex-col gap-4 overflow-y-auto break-words p-4">
              {currentTab === "all" &&
                filterLogs(launchLogsAll).map((log, index) => (
                  <div
                    key={index}
                    className={`${
                      log.includes("ERROR")
                        ? "text-red-500"
                        : log.includes("WARN")
                        ? "text-yellow-500"
                        : log.includes("DEBUG")
                        ? "text-emerald-500"
                        : log.includes("INFO")
                        ? "text-violet-300"
                        : ""
                    }`}
                  >
                    {highlightSearchQuery(log, searchQuery)}
                  </div>
                ))}
              {currentTab === "info" &&
                filterLogs(launchLogsInfo).map((log, index) => (
                  <div key={index} className="">
                    {highlightSearchQuery(log, searchQuery)}
                  </div>
                ))}
              {currentTab === "warn" &&
                filterLogs(launchLogsWarn).map((log, index) => (
                  <div key={index} className="text-yellow-500">
                    {highlightSearchQuery(log, searchQuery)}
                  </div>
                ))}
              {currentTab === "error" &&
                filterLogs(launchLogsError).map((log, index) => (
                  <div key={index} className="text-red-500">
                    {highlightSearchQuery(log, searchQuery)}
                  </div>
                ))}
              {currentTab === "debug" &&
                filterLogs(launchLogsDebug).map((log, index) => (
                  <div key={index} className="text-emerald-500">
                    {highlightSearchQuery(log, searchQuery)}
                  </div>
                ))}
              {currentTab === "component" && (
                <div className="flex flex-col overflow-y-auto break-words p-4">
                  {/* Display logs for the selected package */}
                  {launchLogsComponent
                    .filter((logObj) => logObj.name === selectedPackage)
                    .map((logObj, index) => (
                      <div key={index}>
                        {logObj.logs.map((log, logIndex) => {
                          let logClass = "";
                          if (log.includes("ERROR")) {
                            logClass = "text-red-500";
                          } else if (log.includes("WARN")) {
                            logClass = "text-yellow-500";
                          } else if (log.includes("DEBUG")) {
                            logClass = "text-emerald-500";
                          } else if (log.includes("INFO")) {
                            logClass = "";
                          }

                          return (
                            <div key={logIndex} className={logClass}>
                              {log}
                            </div>
                          );
                        })}
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

function highlightSearchQuery(log: string, query: string) {
  const index = log.toLowerCase().indexOf(query.toLowerCase());
  if (index === -1) return <>{log}</>;

  const beforeQuery = log.substring(0, index);
  const matchedQuery = log.substring(index, index + query.length);
  const afterQuery = log.substring(index + query.length);

  return (
    <>
      {beforeQuery}
      <span className="bg-yellow-300">{matchedQuery}</span>
      {afterQuery}
    </>
  );
}
