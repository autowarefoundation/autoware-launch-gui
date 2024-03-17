"use client";

import { useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { open, save } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";
import { z } from "zod";

import { Button } from "@/components/ui/button";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import {
  autowareFolderPathAtom,
  lastSavedLoadedProfileJSONPathsAtom,
  parsedLaunchFilePathAtom,
  userEditedArgsAtom,
  userEditedBagPlayFlagsAtom,
  userEditedBagRecordFlagsAtom,
} from "@/app/jotai/atoms";

import { Label } from "./ui/label";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "./ui/tooltip";
import { toast } from "./ui/use-toast";

const ProfileDataSchema = z.object({
  autowarePath: z.string(),
  launchFilePath: z.string(),
  userEditedArgs: z.array(
    z.object({
      arg: z.string(),
      value: z.string(),
    })
  ),
  userEditedBagPlayArgs: z.array(
    z.object({
      arg: z.string(),
      value: z.union([z.string(), z.boolean(), z.number()]),
    })
  ),
  userEditedBagRecordArgs: z.array(
    z.object({
      arg: z.string(),
      value: z.union([z.string(), z.boolean(), z.number()]),
    })
  ),
});

const isWindow = typeof window !== undefined;

export function ProfileSetup() {
  const triggerRef = useRef<HTMLButtonElement>(null);
  const [profilePath, setProfilePath] = useState<string>("");
  const [autowarePath, _setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [launchFilePathh, _setLaunchFilePath] = useAtom(
    parsedLaunchFilePathAtom
  );

  const [userEditedArgs, _setUserEditedArgs] = useAtom(userEditedArgsAtom);
  const [userEditedBagPlayArgs, _setUserEditedBagPlayArgs] = useAtom(
    userEditedBagPlayFlagsAtom
  );
  const [userEditedBagRecordArgs, _setUserEditedBagRecordArgs] = useAtom(
    userEditedBagRecordFlagsAtom
  );
  const [pathToLastSavedLoadedProfiles, setPathToLastSavedLoadedProfiles] =
    useAtom(lastSavedLoadedProfileJSONPathsAtom);

  const handleTriggerPopover = () => {
    triggerRef.current?.click();
  };

  const handleSaveProfile = async () => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    const profilePath = await save({
      title: "Save Profile",
      filters: [
        {
          name: "json",
          extensions: ["json"],
        },
      ],
    });

    if (!profilePath) return;

    const addJsonIfNotPresent = (path: string) => {
      if (path.split(".").pop() !== "json") {
        return path + ".json";
      }
      return path;
    };

    setProfilePath(addJsonIfNotPresent(profilePath));

    const profileData = {
      autowarePath,
      launchFilePath: launchFilePathh,
      userEditedArgs,
      userEditedBagPlayArgs,
      userEditedBagRecordArgs,
    };

    //   we send this data to the backend to save the profile in a json file in the profilePath the user chose
    await invoke("save_profile", {
      //   if no extension is provided, add .json to the path
      profilePath: addJsonIfNotPresent(profilePath),
      profileData,
    });

    //   we add the path to the last saved/loaded profiles list
    // we check if the path is already in the list, if it is, we remove it and add it to the top of the list
    if (
      pathToLastSavedLoadedProfiles.includes(addJsonIfNotPresent(profilePath))
    ) {
      const index = pathToLastSavedLoadedProfiles.indexOf(
        addJsonIfNotPresent(profilePath)
      );
      pathToLastSavedLoadedProfiles.splice(index, 1);
    }
    setPathToLastSavedLoadedProfiles([
      addJsonIfNotPresent(profilePath),
      ...pathToLastSavedLoadedProfiles,
    ]);
  };

  const handleLoadProfile = async () => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    const profilePath = await open({
      title: "Load Profile",
      filters: [
        {
          name: "json",
          extensions: ["json"],
        },
      ],
    });

    if (!profilePath) return;

    setProfilePath(profilePath.path);

    //  we send this data to the backend to load the profile from the json file in the profilePath
    //  the user chose
    const rawData = await invoke("load_profile", {
      path: profilePath.path,
    });
    const result = ProfileDataSchema.safeParse(rawData);
    if (!result.success) {
      // Handle the error. The error details are in result.error
      console.error("Invalid profile data:", result.error);
      toast({
        type: "foreground",
        description: "Please check the format of the profile file",
        variant: "destructive",
      });
      return;
    }

    const profileData = result.data;
    const {
      autowarePath,
      launchFilePath,
      userEditedArgs,
      userEditedBagPlayArgs,
      userEditedBagRecordArgs,
    } = profileData;
    _setAutowarePath(autowarePath);
    _setLaunchFilePath(launchFilePath);
    _setUserEditedArgs(userEditedArgs);
    _setUserEditedBagPlayArgs(userEditedBagPlayArgs);
    _setUserEditedBagRecordArgs(userEditedBagRecordArgs);

    if (launchFilePath !== "") {
      await invoke("parse_and_send_xml", {
        path: launchFilePath,
      });
    }
    if (pathToLastSavedLoadedProfiles.includes(profilePath.path)) {
      const index = pathToLastSavedLoadedProfiles.indexOf(profilePath.path);
      pathToLastSavedLoadedProfiles.splice(index, 1);
    }
    setPathToLastSavedLoadedProfiles([
      profilePath.path,
      ...pathToLastSavedLoadedProfiles,
    ]);
  };

  const handleQuickloadProfile = async (profilePath: string) => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    setProfilePath(profilePath);

    const rawData = await invoke("load_profile", {
      path: profilePath,
    });
    const result = ProfileDataSchema.safeParse(rawData);

    if (!result.success) {
      // Handle the error. The error details are in result.error
      console.error("Invalid profile data:", result.error);
      toast({
        type: "foreground",
        description: "Please check the format of the profile file",
        variant: "destructive",
      });
      return;
    }

    const profileData = result.data;
    const {
      autowarePath,
      launchFilePath,
      userEditedArgs,
      userEditedBagPlayArgs,
      userEditedBagRecordArgs,
    } = profileData;
    _setAutowarePath(autowarePath);
    _setLaunchFilePath(launchFilePath);
    _setUserEditedArgs(userEditedArgs);
    _setUserEditedBagPlayArgs(userEditedBagPlayArgs);
    _setUserEditedBagRecordArgs(userEditedBagRecordArgs);

    if (launchFilePath !== "") {
      await invoke("parse_and_send_xml", {
        path: launchFilePath,
      });
    }
    if (pathToLastSavedLoadedProfiles.includes(profilePath)) {
      const index = pathToLastSavedLoadedProfiles.indexOf(profilePath);
      pathToLastSavedLoadedProfiles.splice(index, 1);
    }
    setPathToLastSavedLoadedProfiles([
      profilePath,
      ...pathToLastSavedLoadedProfiles,
    ]);
  };

  return (
    <Popover>
      <PopoverTrigger ref={triggerRef}></PopoverTrigger>

      <Button onClick={handleTriggerPopover} variant="default">
        <TooltipProvider>
          <Tooltip>
            <TooltipTrigger>Profile Load/Save</TooltipTrigger>
            <TooltipContent className="w-60 font-mono text-xs">
              Profiles contain autoware folder path, parsed launch file, and
              user edited args for the launch file as well as user edited bag
              play/record flags
            </TooltipContent>
          </Tooltip>
        </TooltipProvider>
      </Button>
      <PopoverContent className="mr-4 mt-6 w-80">
        <div className="grid gap-4">
          <div className="space-y-2">
            <h4 className="font-medium leading-none">Profiles</h4>
            <p className="text-sm text-muted-foreground">
              Save your most used profiles to easily switch between them.
            </p>
          </div>

          <div className="flex flex-col gap-2">
            <Label className="text-sm font-semibold">Quick load</Label>
            {/* map through the last saved/loaded profiles and show them in a list using Label */}
            {pathToLastSavedLoadedProfiles.map((path) => (
              <Label
                key={path}
                onClick={async () => {
                  handleTriggerPopover();
                  await handleQuickloadProfile(path);
                }}
                className="w-full cursor-pointer rounded-md p-2 hover:bg-secondary-foreground hover:text-secondary"
              >
                {path.split("/").pop()}
              </Label>
            ))}
          </div>

          <div className="flex items-center gap-2">
            <Button
              onClick={async () => {
                handleTriggerPopover();
                await handleSaveProfile();
              }}
              className={"w-fit transition-all duration-300"}
            >
              Save
            </Button>
            <Button
              onClick={async () => {
                handleTriggerPopover();
                await handleLoadProfile();
              }}
              className={"w-fit transition-all duration-300"}
            >
              Load
            </Button>
            <Button
              onClick={async () => {
                handleTriggerPopover();
                setPathToLastSavedLoadedProfiles([]);
              }}
              variant="destructive"
              className={"w-fit transition-all duration-300"}
            >
              Clear all
            </Button>
          </div>
        </div>
      </PopoverContent>
    </Popover>
  );
}
