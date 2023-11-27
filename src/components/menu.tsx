"use client";

import { useEffect, useRef, useState } from "react";
import { useAtom } from "jotai";
import {
  Cable,
  ChevronDown,
  Info,
  Maximize2,
  Minimize2,
  Network,
  Play,
  Rocket,
  Video,
  X,
} from "lucide-react";
import { WindowTitlebar } from "tauri-controls";

import { isBagPlayingAtom, tabAtom } from "@/app/jotai/atoms";

import { AboutDialog } from "./about-dialog";
import SSHComponent from "./SSHDialog";
import { Button } from "./ui/button";
import { Dialog, DialogTrigger } from "./ui/dialog";
import {
  Menubar,
  MenubarContent,
  MenubarItem,
  MenubarMenu,
  MenubarSeparator,
  MenubarShortcut,
  MenubarTrigger,
} from "./ui/menubar";

export function Menu() {
  const [tab, setTab] = useAtom(tabAtom);
  const [isPlaying, _setIsPlaying] = useAtom(isBagPlayingAtom);

  const SSHDialogTriggerRef = useRef<HTMLButtonElement>(null);
  const aboutDialogTriggerRef = useRef<HTMLButtonElement>(null);

  const [isMaximized, setIsMaximized] = useState(false);
  useEffect(() => {
    async function init() {
      const { appWindow } = await import("@tauri-apps/plugin-window");
      const interval = setInterval(() => {
        appWindow.isMaximized().then((maximized) => {
          setIsMaximized(maximized);
        });
      }, 1000);

      return () => {
        clearInterval(interval);
      };
    }
    init();
  }, []);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.ctrlKey) {
        // metaKey is for MacOS (Command key)
        switch (e.key) {
          case "1":
            setTab("launch");
            break;
          case "2":
            setTab("rosbag");
            break;
          case "3":
            setTab("topics");
            break;
          case "`":
            SSHDialogTriggerRef.current?.click();
            break;
          // Add more cases as needed
          default:
            break;
        }
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    // Cleanup the event listener on component unmount
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [setTab]);

  return (
    <WindowTitlebar
      controlsOrder="right"
      className="flex h-10 cursor-grab select-none items-center justify-end gap-2 bg-background pl-4"
      windowControlsProps={{
        platform: "windows",
        className: "w-fit opacity-0 pointer-events-none",
        disabled: true,
      }}
    >
      <Menubar className="border-0">
        <MenubarMenu>
          <MenubarTrigger asChild>
            <Button variant="ghost" className="flex items-center gap-2">
              Menu <ChevronDown className="h-4 w-4" />
            </Button>
          </MenubarTrigger>
          <MenubarContent className="flex flex-col gap-2">
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => {
                SSHDialogTriggerRef.current?.click();
              }}
            >
              <Network className="h-4 w-4" />
              <span className="font-mono text-sm">SSH connection</span>
              <MenubarShortcut>CTRL+`</MenubarShortcut>
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => setTab("launch")}
            >
              <Rocket className="h-4 w-4" />
              <span className="font-mono text-sm">Launch</span>
              <MenubarShortcut>CTRL+1</MenubarShortcut>
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => setTab("rosbag")}
            >
              <Play className="h-4 w-4" />
              <span className="font-mono text-sm">Rosbag Play</span>
              <MenubarShortcut>CTRL+2</MenubarShortcut>
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => setTab("topics")}
            >
              <Video className="h-4 w-4" />
              <span className="font-mono text-sm">Topics / Rosbag Record</span>
              <MenubarShortcut>CTRL+3</MenubarShortcut>
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => {
                aboutDialogTriggerRef.current?.click();
              }}
            >
              <Info className="h-4 w-4" />
              <span className="font-mono text-sm">About</span>
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={async () => {
                const { appWindow } = await import("@tauri-apps/plugin-window");
                if (await appWindow.isMaximized()) {
                  appWindow.unmaximize();
                } else {
                  appWindow.maximize();
                }
              }}
            >
              {isMaximized ? (
                <>
                  <Minimize2 className="h-4 w-4" />
                  <span className="font-mono text-sm">Minimize</span>
                </>
              ) : (
                <>
                  <Maximize2 className="h-4 w-4" />
                  <span className="font-mono text-sm">Maximize</span>
                </>
              )}
            </MenubarItem>
            <MenubarSeparator />
            <MenubarItem
              className="flex items-center gap-2"
              onClick={() => window.close()}
            >
              <X className="h-4 w-4" />
              <span className="font-mono text-sm">Quit</span>
              <MenubarShortcut>CTRL+Q</MenubarShortcut>
            </MenubarItem>
          </MenubarContent>
        </MenubarMenu>
      </Menubar>

      <Dialog modal={false}>
        <DialogTrigger ref={SSHDialogTriggerRef}></DialogTrigger>
        <SSHComponent />
      </Dialog>

      <Dialog modal={false}>
        <DialogTrigger ref={aboutDialogTriggerRef}></DialogTrigger>
        <AboutDialog />
      </Dialog>
      {/* App Title */}
      <h1 className="pointer-events-none ml-auto font-sans text-lg font-semibold">
        Autoware Launch GUI
      </h1>
    </WindowTitlebar>
  );
}
