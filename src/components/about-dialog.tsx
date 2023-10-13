"use client";

import { useCallback, useState } from "react";
import { UpdateIcon } from "@radix-ui/react-icons";
import { GithubIcon, HomeIcon } from "lucide-react";

import { Icons } from "./icons";
import { Button } from "./ui/button";
import {
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "./ui/dialog";

export function AboutDialog() {
  const [updateText, setUpdateText] = useState("");
  const [version, setVersion] = useState("");
  const [name, setName] = useState("");
  const [tauriVersion, setTauriVersion] = useState("");
  const [arc, setArc] = useState("");

  const getInfos = useCallback(async () => {
    const { getName, getTauriVersion, getVersion } = await import(
      "@tauri-apps/plugin-app"
    );
    const { arch } = await import("@tauri-apps/plugin-os");

    getName && getName().then((x) => setName(x));
    getVersion && getVersion().then((x) => setVersion(x));
    getTauriVersion && getTauriVersion().then((x) => setTauriVersion(x));
    arch && arch().then((x) => setArc(x));
  }, []);

  if (typeof window !== "undefined") getInfos();

  const open = useCallback(async (url: string) => {
    const { open } = await import("@tauri-apps/plugin-shell");
    open && open(url);
  }, []);

  return (
    <DialogContent className="overflow-clip pb-2">
      <DialogHeader className="flex items-center text-center">
        <div className="bg-background">
          <Icons.logo className="invert dark:invert-0" />
        </div>

        <DialogTitle className="flex flex-col items-center gap-2 pt-2">
          {name}
          <span className="flex gap-1 font-mono text-xs font-medium">
            Version {version} ({arc})
            <span className="font-sans font-medium text-gray-400">
              (
              <span
                className="cursor-pointer text-blue-500"
                onClick={() =>
                  open(
                    "https://github.com/leo-drive/autoware-launch-gui/releases/"
                  )
                }
              >
                release notes
              </span>
              )
            </span>
          </span>
        </DialogTitle>

        <DialogDescription className=" text-foreground">
          Launch GUI for Autoware Processes
        </DialogDescription>

        <span className="text-xs text-gray-400">{updateText}</span>
        {/* <DialogDescription className="flex flex-row"></DialogDescription> */}
      </DialogHeader>

      <span className="font-mono text-xs font-medium text-gray-400">
        Tauri version: {tauriVersion}
      </span>

      <DialogFooter className="flex flex-row items-center border-t pt-2 text-slate-400">
        <div className="mr-auto flex flex-row gap-2">
          <HomeIcon
            className="h-5 w-5 cursor-pointer transition hover:text-slate-300"
            onClick={() => open("https://autoware.org/")}
          />
          <GithubIcon
            className="h-5 w-5 cursor-pointer transition hover:text-slate-300 "
            onClick={() =>
              open("https://github.com/leo-drive/autoware-launch-gui")
            }
          />
        </div>

        <Button
          type="submit"
          variant="outline"
          className="h-7 gap-1"
          onClick={() => setUpdateText("You have the latest version.")}
        >
          <UpdateIcon /> Check for Updates
        </Button>
      </DialogFooter>
    </DialogContent>
  );
}
