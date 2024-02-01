"use client";

import { useEffect, useState } from "react";
import { disable, enable, isEnabled } from "@tauri-apps/plugin-autostart";
import { useAtom } from "jotai";
import { HomeIcon, Rewind, X } from "lucide-react";

import {
  autowareFolderPathAtom,
  multipleWorkspacePathsAtom,
} from "@/app/jotai/atoms";

import { Icons } from "../icons";
import { ModeToggle } from "../mode-toggle";
import { Button } from "../ui/button";
import { Label } from "../ui/label";

const Settings = () => {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );

  const [extraWorkspaces, setExtraWorkspaces] = useAtom(
    multipleWorkspacePathsAtom
  );

  const addWorkspace = async () => {
    const { open } = await import("@tauri-apps/plugin-dialog");

    const result = await open({
      directory: true,
      multiple: false,
    });
    console.log(result);
    if (result) {
      setExtraWorkspaces((currentWorkspaces) => {
        // without the use of a set, let's skip duplicates
        if (currentWorkspaces.includes(result)) return currentWorkspaces;
        return [...currentWorkspaces, result];
      });
    }
  };

  const removeWorkspace = async (workspace: string) => {
    setExtraWorkspaces((workspaces) => {
      return workspaces.filter((w) => w !== workspace);
    });
  };

  const [isAutostartEnabled, setIsAutostartEnabled] = useState<boolean>(false);

  const toggleAutostart = async () => {
    if (await isEnabled()) {
      await disable();
      setIsAutostartEnabled(false);
    } else {
      await enable();
      setIsAutostartEnabled(true);
    }
  };

  useEffect(() => {
    (async () => {
      setIsAutostartEnabled(await isEnabled());
    })();
  }, []);

  return (
    <div className="flex h-full w-full flex-col items-center justify-center gap-4 p-2">
      {/* this div will wrap the centered items and give them a border */}
      <div className="flex w-3/4 flex-col gap-4 rounded-xl  p-4 shadow-[0_0_120px_rgba(8,_112,_184,_0.7)]">
        <div>
          <div className="flex items-center gap-2 bg-background">
            <Icons.logo className="h-16 w-16 fill-current text-foreground" />
            <Label className="text-sm font-semibold text-foreground">
              Autoware Launch GUI Settings
            </Label>
          </div>
        </div>
        <Label className="text-sm text-muted-foreground">
          <span className="font-semibold">Autoware</span> is an open source
          project hosted by the{" "}
          <a
            className="underline"
            href="https://www.autoware.org/"
            target="_blank"
            rel="noopener noreferrer"
          >
            Autoware Foundation
          </a>
          .
        </Label>
        <Label className="text-sm text-muted-foreground">
          Autoware Workspace Path:{" "}
          <span className="font-mono">{autowareFolderPath}</span>
        </Label>
        <Label className="text-xs text-foreground">
          Additional Workspaces to source from on all functionality, aside from
          the current autoware workspace:
        </Label>
        <div className="flex max-h-[8rem] min-h-[8rem]  flex-col gap-2 overflow-y-auto rounded-md border p-2">
          {extraWorkspaces.map((workspace, index) => (
            <div
              key={`${workspace}+${index}`}
              className="flex w-full max-w-full items-center justify-between rounded-md p-2 font-mono text-muted-foreground hover:bg-foreground hover:bg-opacity-10 hover:text-background"
            >
              <div className="flex w-4/5 items-center gap-2">
                <HomeIcon className="h-4 w-4 fill-current" />
                <div className="flex max-w-[80%] flex-row items-center gap-2">
                  <Label className="truncate text-sm font-semibold">
                    {workspace}
                  </Label>
                </div>
              </div>
              <div className="flex w-4 items-center gap-2">
                <X
                  className="h-4 w-4 cursor-pointer fill-current"
                  onClick={async () => await removeWorkspace(workspace)}
                />
              </div>
            </div>
          ))}
        </div>
        <Button
          className="flex flex-row items-center gap-2 rounded-md p-2 font-mono text-muted-foreground hover:bg-foreground hover:bg-opacity-10 hover:text-background"
          onClick={addWorkspace}
          variant="outline"
        >
          <Icons.add className="h-4 w-4 fill-current" />
          <Label className="text-sm font-semibold">Add Workspace</Label>
        </Button>
        <div className="flex items-center gap-2">
          <Label className="text-xs font-semibold text-foreground">
            Change Theme:
          </Label>
          <ModeToggle />
        </div>
        <div className="flex items-center gap-2">
          <Label className="text-xs font-semibold text-foreground">
            Autostart:
          </Label>
          <Button
            className="flex flex-row items-center gap-2 rounded-md p-2 font-mono text-muted-foreground hover:bg-foreground hover:bg-opacity-10 hover:text-background"
            onClick={toggleAutostart}
            variant="outline"
          >
            <Rewind className="h-4 w-4 fill-current" />
            <Label className="text-sm font-semibold">
              {isAutostartEnabled ? "Disable" : "Enable"}
            </Label>
          </Button>
        </div>
      </div>
    </div>
  );
};

export default Settings;
