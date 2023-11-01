"use client";

import React, { useRef, useState } from "react";
import { open } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";

import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import {
  autowareFolderPathAtom,
  lastUsedAutowareFoldersAtom,
} from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import { Label } from "./ui/label";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "./ui/tooltip";

const AutowareFolderSetup = () => {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );
  const [lastUsedFolders, setLastUsedFolders] = useAtom(
    lastUsedAutowareFoldersAtom
  );

  const triggerRef = useRef<HTMLButtonElement>(null);

  const handleTriggerPopover = () => {
    triggerRef.current?.click();
  };

  const quickSetPath = (path: string) => {
    setAutowareFolderPath(path);
  };

  return (
    <Popover>
      <TooltipProvider>
        <Tooltip>
          <TooltipTrigger asChild>
            <Button
              onClick={handleTriggerPopover}
              className="w-40 text-left underline underline-offset-1"
              variant={autowareFolderPath ? "ghost" : "default"}
            >
              <span className="truncate font-mono">
                {autowareFolderPath
                  ? autowareFolderPath.split("/").slice(-2).join("/")
                  : "Autoware Folder Path"}
              </span>
            </Button>
          </TooltipTrigger>
          <TooltipContent>
            <div className="font-mono text-sm">
              <div>
                Please select the Autoware root folder eg. /home/user/autoware.
              </div>
              <div>Current Workspace is : {autowareFolderPath}</div>
            </div>
          </TooltipContent>
        </Tooltip>
      </TooltipProvider>
      <PopoverTrigger ref={triggerRef}></PopoverTrigger>
      <PopoverContent className="w-80">
        <div className="grid gap-4">
          <div className="space-y-4">
            <Label className="font-mono font-medium leading-none">
              Last Autoware Workspaces
            </Label>
            <ul className="flex flex-col items-start gap-2">
              {lastUsedFolders.map((folder, index) => (
                <Label
                  className="w-full cursor-pointer rounded-md p-2 hover:bg-secondary-foreground hover:text-secondary"
                  key={index}
                  onClick={() => {
                    quickSetPath(folder);
                  }}
                >
                  {folder}
                </Label>
              ))}
            </ul>
          </div>
          <div className="flex items-center gap-2">
            <Button
              onClick={async () => {
                const folder = await open({
                  directory: true,
                  multiple: false,
                  title:
                    "Select Autoware Root Folder [eg. /home/user/autoware]",
                });
                if (!folder) return;
                setAutowareFolderPath(folder as string);
                setLastUsedFolders((prev) => {
                  if (!prev.includes(folder as string)) {
                    return [folder as string, ...prev];
                  }
                  return prev;
                });
              }}
              className={"w-fit"}
            >
              Select root folder
            </Button>
            <Button
              onClick={() => {
                setLastUsedFolders([]);
              }}
              variant="destructive"
            >
              Clear
            </Button>
          </div>
        </div>
      </PopoverContent>
    </Popover>
  );
};

export default AutowareFolderSetup;
