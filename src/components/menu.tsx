"use client";

import { useCallback } from "react";
import { DialogTrigger } from "@radix-ui/react-dialog";
import { WindowTitlebar } from "tauri-controls";

import { AboutDialog } from "./about-dialog";
import { Dialog } from "./ui/dialog";

export function Menu() {
  const closeWindow = useCallback(async () => {
    const { appWindow } = await import("@tauri-apps/plugin-window");
    appWindow.close();
  }, []);

  return (
    <WindowTitlebar
      controlsOrder="right"
      className="flex h-10 cursor-grab select-none items-center justify-end bg-background pl-4"
      windowControlsProps={{ platform: "windows", className: "w-fit" }}
    >
      <Dialog modal={false}>
        <DialogTrigger>About</DialogTrigger>
        <AboutDialog />
      </Dialog>
      <h1 className="pointer-events-none ml-auto font-sans text-lg font-semibold">
        Autoware Launch GUI
      </h1>
    </WindowTitlebar>
  );
}
