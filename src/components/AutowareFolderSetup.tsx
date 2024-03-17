"use client";

import React, { useCallback, useEffect, useRef, useState } from "react";
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

const isWindow = typeof window !== undefined;

const AutowareFolderSetup = () => {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );
  const [lastUsedFolders, setLastUsedFolders] = useAtom(
    lastUsedAutowareFoldersAtom
  );
  const [socket, setSocket] = useState<WebSocket | null>(null);

  useEffect(() => {
    let ws: WebSocket;
    if (isWindow) {
      ws = new WebSocket("ws://localhost:42068/ws");
      ws.onopen = () => {
        console.log("connected to app/browser syncing websocket");

        // @ts-ignore
        if (window.__TAURI__) {
          ws.send("Tauri-FolderSetup");
        } else {
          ws.send("Browser-FolderSetup");
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

      setSocket(ws);
    }

    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [autowareFolderPath]);

  const triggerRef = useRef<HTMLButtonElement>(null);

  const handleTriggerPopover = () => {
    triggerRef.current?.click();
  };

  const quickSetPath = (path: string) => {
    setAutowareFolderPath(path);
    updateBrowser(path);
  };
  // need to confirm if autowarefolder has been set
  const updateBrowser = useCallback(
    (autowareFolder: string) => {
      // @ts-ignore
      if (window.__TAURI__ && socket) {
        socket.send(
          JSON.stringify({
            autowareFolderPath: autowareFolder,
          })
        );
      }
    },
    [socket]
  );

  return (
    <Popover>
      <TooltipProvider>
        <Tooltip>
          <TooltipTrigger asChild>
            <Button
              onClick={handleTriggerPopover}
              className="w-40 text-left underline underline-offset-1"
              variant={autowareFolderPath ? "ghost" : "default"}
              // @ts-ignore
              disabled={window.__TAURI__ ? false : true}
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
                // @ts-ignore
                if (!(isWindow && window.__TAURI__)) {
                  return;
                }
                const folder = await open({
                  directory: true,
                  multiple: false,
                  title:
                    "Select Autoware Root Folder [eg. /home/user/autoware]",
                });
                if (!folder) return;
                setAutowareFolderPath(folder);
                updateBrowser(folder);
                setLastUsedFolders((prev) => {
                  if (!prev.includes(folder)) {
                    return [folder, ...prev];
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
