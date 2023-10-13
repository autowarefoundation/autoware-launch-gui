import { useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { message, open, save } from "@tauri-apps/plugin-dialog";
import { readTextFile } from "@tauri-apps/plugin-fs";
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
} from "@/app/jotai/atoms";

import { Label } from "./ui/label";

const ProfileDataSchema = z.object({
  autowarePath: z.string(),
  launchFilePath: z.string(),
  userEditedArgs: z.array(
    z.object({
      arg: z.string(),
      value: z.string(),
    })
  ),
});

export function ProfileSetup() {
  const triggerRef = useRef<HTMLButtonElement>(null);
  const [profilePath, setProfilePath] = useState<string>("");
  const [autowarePath, _setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [launchFilePathh, _setLaunchFilePath] = useAtom(
    parsedLaunchFilePathAtom
  );

  const [userEditedArgs, _setUserEditedArgs] = useAtom(userEditedArgsAtom);
  const [pathToLastSavedLoadedProfiles, setPathToLastSavedLoadedProfiles] =
    useAtom(lastSavedLoadedProfileJSONPathsAtom);

  const handleTriggerPopover = () => {
    triggerRef.current?.click();
  };

  const handleSaveProfile = async () => {
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
    };

    //   we send this data to the backend to save the profile in a json file in the profilePath the user chose
    await invoke("save_profile", {
      payload: {
        //   if no extension is provided, add .json to the path
        profilePath: addJsonIfNotPresent(profilePath),
        profileData,
      },
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
      payload: {
        path: profilePath.path,
      },
    });
    const result = ProfileDataSchema.safeParse(rawData);
    if (!result.success) {
      // Handle the error. The error details are in result.error
      console.error("Invalid profile data:", result.error);
      message("Please check the format of the profile file", {
        title: "Invalid profile data",
        type: "error",
      });
      return;
    }

    const profileData = result.data;
    const { autowarePath, launchFilePath, userEditedArgs } = profileData;
    _setAutowarePath(autowarePath);
    _setLaunchFilePath(launchFilePath);
    _setUserEditedArgs(userEditedArgs);

    if (launchFilePath !== "") {
      await invoke("parse_and_send_xml", {
        payload: {
          path: launchFilePath,
        },
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
    setProfilePath(profilePath);

    const rawData = await invoke("load_profile", {
      payload: {
        path: profilePath,
      },
    });
    const result = ProfileDataSchema.safeParse(rawData);

    if (!result.success) {
      // Handle the error. The error details are in result.error
      console.error("Invalid profile data:", result.error);
      message("Please check the format of the profile file", {
        title: "Invalid profile data",
        type: "error",
      });
      return;
    }

    const profileData = result.data;
    const { autowarePath, launchFilePath, userEditedArgs } = profileData;
    _setAutowarePath(autowarePath);
    _setLaunchFilePath(launchFilePath);
    _setUserEditedArgs(userEditedArgs);

    if (launchFilePath !== "") {
      await invoke("parse_and_send_xml", {
        payload: {
          path: launchFilePath,
        },
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
        Profile Load/Save
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
