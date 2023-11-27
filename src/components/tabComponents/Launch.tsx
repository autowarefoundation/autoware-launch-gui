"use client";

import React, { useCallback, useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { open } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  autowareProcessesAtom,
  launchLogsAllAtom,
  launchLogsComponentAtom,
  launchLogsDebugAtom,
  launchLogsErrorAtom,
  launchLogsInfoAtom,
  launchLogsWarnAtom,
  parsedLaunchFilePathAtom,
  parsedLaunchFilesAtom,
  pidsLengthAtom,
  selectedLaunchArgsAtom,
  sshHostAtom,
  sshIsConnectedAtom,
  sshPasswordAtom,
  sshUsernameAtom,
  userEditedArgsAtom,
} from "@/app/jotai/atoms";

import AutowareFolderSetup from "../AutowareFolderSetup";
import { AutowareLaunchDialog } from "../AutowareLaunchDialog";
import CalibrationTools from "../CalibrationTools";
import { EditArgsDialog } from "../EditArgsDialog";
import { ProfileSetup } from "../ProfileSetup";
import type { ElementData } from "../Tree";
import { Button } from "../ui/button";
import { Input } from "../ui/input";
import { Label } from "../ui/label";
import { useToast } from "../ui/use-toast";
import YAMLEdit from "./YamlEdit";

const Launch = () => {
  const [pidsLen, setPidsLen] = useAtom(pidsLengthAtom);
  const [elements, setElements] = useAtom(parsedLaunchFilesAtom);
  const [_launchFilePath, _setLaunchFilePath] = useAtom(
    parsedLaunchFilePathAtom
  );
  const { toast } = useToast();

  useEffect(() => {
    async function init() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      // TODO: Figure out the tauri-controls situation
      const unlistenCloseRequest = await appWindow.listen(
        "close_requested",
        async (msg) => {
          await invoke("kill_autoware_process", {
            payload: {},
          });

          appWindow.close();
        }
      );

      const unlistenPidsLen = await appWindow.listen("pids_len", (msg) => {
        setPidsLen(msg.payload as number);
      });

      const unlistenPidsCleared = await appWindow.listen(
        "pids-cleared",
        (msg) => {
          setPidsLen(0);
        }
      );

      const unlistenAutowareLaunchFileParsed = await appWindow.listen(
        "receiveTree",
        (msg) => {
          setElements(
            JSON.parse(msg.payload as string).elements as ElementData[]
          );
        }
      );

      const unlistenPackageNotFound = await appWindow.listen(
        "package-not-found",
        () => {
          toast({
            title: "Caught an Exception",
            description:
              "Exception hit, please check the error logs or try running the app from the terminal `autoware-launch-gui`",
            variant: "destructive",
          });
        }
      );

      return () => {
        unlistenCloseRequest();
        unlistenPidsLen();
        unlistenPidsCleared();
        unlistenAutowareLaunchFileParsed();
        unlistenPackageNotFound();
      };
    }

    init();
  }, []);

  const [parsedFilePath, setParsedFilePath] = useAtom(parsedLaunchFilePathAtom);
  const [launchFilePath, setLaunchFilePath] = useState(
    parsedFilePath?.split("/").pop()
  );

  useEffect(() => {
    setLaunchFilePath(parsedFilePath?.split("/").pop());
  }, [parsedFilePath]);

  const [editableCommand, setEditableCommand] = useState(
    `ros2 launch autoware_launch ${launchFilePath}`
  );
  const [selectedArgs, setSelectedArgs] = useAtom(selectedLaunchArgsAtom);
  const [userEditedArgs, setUserEditedArgs] = useAtom(userEditedArgsAtom);

  const [autowarePath] = useAtom(autowareFolderPathAtom);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const [autowareProcessesNames, _setAutowareProcessesNames] = useAtom(
    autowareProcessesAtom
  );

  const [parsedLaunchFile, _] = useAtom(parsedLaunchFilesAtom);

  // get all the arguments from the tree
  const args: {
    arg: string;
    value: string;
  }[] = [];

  const walkTreeItems = (items: ElementData[] | ElementData) => {
    if (items instanceof Array) {
      for (let i = 0; i < items.length; i++) {
        if (items[i]!.name === "arg") {
          const arg = items[i]!.attributes.find((attr) => attr[0] === "name");
          const defaultValue = items[i]!.attributes.find(
            (attr) => attr[0] === "default"
          );
          if (arg) {
            // check if the argument is already in the args array
            // if not, add it
            // this is to prevent duplicate arguments
            if (!args.find((param) => param.arg === arg[1]))
              args.push({
                arg: arg[1],
                value: defaultValue ? defaultValue[1] : "no default value",
              });
          }
        }
        walkTreeItems(items[i]!);
      }
    } else if (items.children) {
      walkTreeItems(items.children);
    }
  };

  walkTreeItems(elements);

  useEffect(() => {
    setEditableCommand(`ros2 launch autoware_launch ${launchFilePath}`);
  }, [launchFilePath]);

  const handleArgSelectChange = React.useCallback(
    (arg: { arg: string; value: string }) => {
      let updatedArgs: { arg: string; value: string }[] = [];
      let updatedCommand = editableCommand;

      // if the argument is already selected, update the selectedArgs array and update the editableCommand
      // if not, add the argument to the selectedArgs array and update the editableCommand
      // don't remove it just update the values

      if (selectedArgs.find((selectedArg) => selectedArg.arg === arg.arg)) {
        updatedArgs = selectedArgs.map((selectedArg) => {
          if (selectedArg.arg === arg.arg) {
            return arg;
          }
          return selectedArg;
        });

        // Update the argument in the editableCommand based on its name
        const argPattern = new RegExp(`\\b${arg.arg}:=\\S+`, "g");
        updatedCommand = updatedCommand.replace(
          argPattern,
          `${arg.arg}:=${arg.value}`
        );
      } else {
        updatedArgs = [...selectedArgs, arg];
        updatedCommand += ` ${arg.arg}:=${arg.value}`;
      }

      // Merge userEditedArgs with updatedArgs
      const mergedArgs = [...userEditedArgs];
      updatedArgs.forEach((updatedArg) => {
        const index = mergedArgs.findIndex(
          (mergedArg) => mergedArg.arg === updatedArg.arg
        );
        if (index !== -1) {
          mergedArgs[index] = updatedArg; // Override with selectedArg
        } else {
          mergedArgs.push(updatedArg);
        }
      });

      setEditableCommand(updatedCommand);
      setSelectedArgs(updatedArgs);

      // push cursor to the end of the input field
      setTimeout(() => {
        textareaRef.current?.focus();
      }, 100);
    },
    [selectedArgs, userEditedArgs, editableCommand]
  );

  const handleArgRemove = React.useCallback(
    (arg: { arg: string; value: string }) => {
      let updatedArgs: { arg: string; value: string }[] = [];
      let updatedCommand = editableCommand;

      // if the argument is already selected, remove it from the selectedArgs array and update the editableCommand so it doesn't show up
      // if not, do nothing

      if (selectedArgs.find((selectedArg) => selectedArg.arg === arg.arg)) {
        updatedArgs = selectedArgs.filter((selectedArg) => {
          return selectedArg.arg !== arg.arg;
        });

        // Update the argument in the editableCommand based on its name
        const argPattern = new RegExp(`\\b${arg.arg}:=\\S+`, "g");
        updatedCommand = updatedCommand.replace(argPattern, "");
      } else {
        return;
      }

      // Merge userEditedArgs with updatedArgs
      const mergedArgs = [...userEditedArgs];
      updatedArgs.forEach((updatedArg) => {
        const index = mergedArgs.findIndex(
          (mergedArg) => mergedArg.arg === updatedArg.arg
        );
        if (index !== -1) {
          mergedArgs[index] = updatedArg; // Override with selectedArg
        } else {
          mergedArgs.push(updatedArg);
        }
      });

      setEditableCommand(updatedCommand);
      setSelectedArgs(updatedArgs);

      // push cursor to the end of the input field
      setTimeout(() => {
        textareaRef.current?.focus();
      }, 100);
    },
    [selectedArgs, userEditedArgs, editableCommand]
  );
  // This effect updates userEditedArgs when the user manually edits the editableCommand input field // but not needed since it became read only
  // useEffect(() => {
  //   const argsFromCommand = editableCommand
  //     .split(" ")
  //     .slice(4)
  //     .map((arg) => {
  //       const [argName, argValue] = arg.split(":=");
  //       return {
  //         arg: argName,
  //         value: argValue,
  //       };
  //     })
  //     .filter((arg) => arg.arg && arg.value);

  //   const baseCommand = `ros2 launch autoware_launch ${launchFilePath}`;
  //   // we check if for whatever reason the user manually edited the command and removed the full path to the launch file
  //   // if so, we add it back and make sure nothing can be added before it by using indexOf
  //   if (editableCommand.indexOf(baseCommand) !== 0) {
  //     setEditableCommand(baseCommand);
  //   }
  //   setUserEditedArgs(argsFromCommand);
  // }, [editableCommand]);

  const [_launchLogsAll, setLaunchLogsAll] = useAtom(launchLogsAllAtom);
  const [_launchLogsInfo, setLaunchLogsInfo] = useAtom(launchLogsInfoAtom);
  const [_launchLogsWarn, setLaunchLogsWarn] = useAtom(launchLogsWarnAtom);
  const [_launchLogsError, setLaunchLogsError] = useAtom(launchLogsErrorAtom);
  const [_launchLogsDebug, setLaunchLogsDebug] = useAtom(launchLogsDebugAtom);
  const [_launchLogsComponent, setLaunchLogsComponent] = useAtom(
    launchLogsComponentAtom
  );

  const [host, _setHost] = useAtom(sshHostAtom);
  const [user, _setUser] = useAtom(sshUsernameAtom);
  const [isSSHConnected, _setIsConnected] = useAtom(sshIsConnectedAtom);

  const handleLaunchAutoware = useCallback(async () => {
    if (!autowarePath) {
      toast({
        type: "foreground",
        description: "Please select autoware folder path",
        variant: "destructive",
      });
      return;
    }
    if (!launchFilePath) {
      toast({
        type: "foreground",
        description: "Please select a launch file",
        variant: "destructive",
      });
      return;
    }

    const argsToLaunch = userEditedArgs;

    // We need to check the argsToLaunch for these 3 arguments as a minimum requirement
    // to be able to run the invoke function
    // map_path, vehicle_model, and sensor_model
    const requiredArgs = ["map_path", "vehicle_model", "sensor_model"] as const;

    if (
      // use the requiredArgs array to check if the argsToLaunch array contains
      // the required arguments
      !requiredArgs.every((requiredArg) =>
        argsToLaunch.find((arg) => arg.arg === requiredArg)
      )
    ) {
      toast({
        variant: "destructive",
        title: "Error",
        description:
          "Please select map_path, vehicle_model, and sensor_model at the very least to be able to launch Autoware",
      });
      return;
    }

    // read the contents at the map_path argument
    const mapPathArg = argsToLaunch.find((arg) => arg.arg === "map_path");

    if (mapPathArg) {
      const mapPath = mapPathArg.value;
      const mapPathContent = (await invoke("files_in_dir", {
        payload: {
          mapPath: mapPath,
        },
      })) as string[];

      // we need to only keep the filename with the extension
      // and remove the path to the file
      const mapPathContentFiles = mapPathContent.map((file) => {
        return file.split("/").pop();
      });

      const mapFileAttributes = parsedLaunchFile
        .map((item) => {
          const attributes = item.children?.map((child) => {
            const smth = child.attributes.find((attr) => {
              return (
                attr[0] === "name" &&
                (attr[1] === "lanelet2_map_file" ||
                  attr[1] === "pointcloud_map_file")
              );
            });

            // lets return the full attribute array of the child if it has the name "lanelet2_map_file" or "pointcloud_map_file"

            if (smth) {
              return child.attributes;
            }
          });

          // filter out undefined values and return all the attributes that have the name "lanelet2_map_file" or "pointcloud_map_file"
          return attributes?.filter((attr) => attr !== undefined);
        })
        .flat();

      // now we check if the mapPathContent array contains any of the mapFileAttributes default values
      // if so, we show a toast to the user
      const mapFileAttributesValues = mapFileAttributes.map((attr) => {
        return attr?.find((attr) => attr[0] === "default")?.[1];
      });

      const mapFileAttributesValuesFound = mapFileAttributesValues.filter(
        (attr) => {
          return mapPathContentFiles.includes(attr);
        }
      );

      // at this point if mapFileAttributesValuesFound has both values of lanelet2_map_file and pointcloud_map_file
      // we can assume that the user has the correct map files in the map_path directory
      // and we can launch autoware
      if (mapFileAttributesValuesFound.length === 2) {
        // reset the launch logs
        setLaunchLogsAll([]);
        setLaunchLogsInfo([]);
        setLaunchLogsWarn([]);
        setLaunchLogsError([]);
        setLaunchLogsDebug([]);
        setLaunchLogsComponent([]);

        if (!isSSHConnected) {
          await invoke("launch_autoware", {
            payload: {
              path: autowarePath,
              launchFile: launchFilePath,
              argsToLaunch,
            },
          });
        } else {
          // Construct the command arguments string dynamically
          const cmdArgs = argsToLaunch
            .map((arg) => `${arg.arg}:=${arg.value}`)
            .join(" ");

          const cmdStr =
            `source /opt/ros/humble/setup.bash && ` +
            `source /home/${user}/autoware/install/setup.bash && ` +
            `ros2 launch autoware_launch ${launchFilePath} ${cmdArgs}`;

          setAutowareLaunchedInSSH(true);
          const result = (await invoke("execute_command_in_shell", {
            payload: { command: cmdStr, user, host },
          })) as string;
        }
        return;
      } else {
        toast({
          variant: "destructive",
          title: "Error",
          description: `Please make sure you have the map files in the map_path directory`,
        });
      }
    }
  }, [autowarePath, launchFilePath, parsedLaunchFile, userEditedArgs]);

  const handleParseLaunchFile = useCallback(async () => {
    const file = await open({
      // xml files
      defaultPath:
        `${autowarePath}/src/launcher/autoware_launch/autoware_launch/launch` ||
        undefined,
      filters: [
        {
          name: "xml",
          extensions: ["xml"],
        },
      ],
      title: "Select A Launch File",
    });
    if (!file && !parsedFilePath) {
      toast({
        type: "foreground",
        description: "Please select a launch file",
        variant: "destructive",
      });
    }
    if (!file) {
      return;
    }
    await invoke("parse_and_send_xml", {
      payload: {
        path: file.path,
        calibrationTool: false,
      },
    });

    // const fileNameWithExtension = file.path.split("/").pop();
    setParsedFilePath(file.path);
    setUserEditedArgs([]);
  }, [autowarePath]);

  const [autowareLaunchedInSSH, setAutowareLaunchedInSSH] = useState(false);

  return (
    <div className="flex h-full w-full flex-col gap-4">
      <div className="flex flex-row gap-4">
        <AutowareLaunchDialog />
        <Button
          onClick={async () => {
            await handleParseLaunchFile();
          }}
          className="w-fit underline underline-offset-1"
          variant="ghost"
          disabled={!autowarePath}
        >
          {launchFilePath ? launchFilePath : "Select A Launch File"}
        </Button>
        <Button
          onClick={async () => {
            await handleLaunchAutoware();
          }}
          disabled={
            pidsLen !== 0 ||
            !autowarePath ||
            !launchFilePath ||
            autowareProcessesNames.length > 35 ||
            (autowareLaunchedInSSH && isSSHConnected)
          }
        >
          {autowareProcessesNames.length > 35
            ? "Autoware is running"
            : pidsLen !== 0
            ? "Launching Autoware"
            : "Launch Autoware"}
        </Button>
        <Button
          onClick={async () => {
            if (!isSSHConnected) {
              await invoke("kill_autoware_process", {
                payload: {},
              });
            } else {
              const response = await invoke("kill_ssh_connection", {
                payload: {},
              });
              _setIsConnected(false);
              setAutowareLaunchedInSSH(false);

              toast({
                variant: "destructive",
                title: "Autoware Killed in Remote Machine",
                description:
                  "Autoware is closed and SSH Connection Closed Successfully",
              });
            }
          }}
          className="w-fit"
          disabled={
            (!isSSHConnected && pidsLen === 0) ||
            (!autowareLaunchedInSSH && isSSHConnected)
          }
          variant="destructive"
        >
          Kill Autoware
        </Button>

        <AutowareFolderSetup />

        <div className="ml-auto flex items-center gap-2">
          <CalibrationTools />
          <ProfileSetup />
        </div>
      </div>
      {/* <Tree data={elements} /> */}
      <div className="flex w-full flex-row gap-4">
        <div className="flex w-full flex-col gap-4">
          <h1 className="text-lg font-semibold">
            {parsedFilePath.split("/").pop()?.split(".")[0]} parameters
          </h1>
          {args.length > 0 && (
            <div className="flex h-60 w-full flex-col gap-2 overflow-y-auto rounded-lg border p-2">
              {args.map((arg, idx) => (
                <div
                  key={arg.arg + idx}
                  className="mb-2 flex items-center justify-between gap-4"
                >
                  <div className="flex items-center">
                    <Label htmlFor={arg.arg + idx} className="ml-2">
                      {arg.arg}
                    </Label>
                  </div>
                  <EditArgsDialog
                    handleArgSelect={handleArgSelectChange}
                    handleArgRemove={handleArgRemove}
                    arg={arg}
                  />
                </div>
              ))}
            </div>
          )}
        </div>
        <div className="flex w-full flex-col gap-4">
          {/* <Textarea
          readOnly
          ref={textareaRef}
          value={editableCommand}
          onChange={(e) => {
            setEditableCommand(e.target.value);
          }}
          placeholder="Your ros2 launch command here"
          className="h-[10rem] w-full resize-none"
        /> */}
          <span className="font-semibold">
            Arguments added to launch command
          </span>
          <div className="flex h-60 w-full flex-col gap-2 overflow-y-auto rounded-lg p-2">
            {userEditedArgs.map((arg, idx) => (
              <div
                key={arg.arg + idx}
                className="relative z-0 mb-2 flex w-full items-center justify-between gap-4"
              >
                <Label
                  htmlFor="floating_outlined"
                  className={`absolute left-1 top-2 z-10 origin-[0] -translate-y-4 scale-75 transform bg-inherit px-2 text-sm opacity-100 transition-all
                  duration-300 peer-placeholder-shown:top-1/2 peer-placeholder-shown:-translate-y-1/2 peer-placeholder-shown:scale-100 peer-focus:top-2 peer-focus:-translate-y-4 peer-focus:scale-75 peer-focus:px-2`}
                >
                  {arg.arg}
                </Label>
                <Input
                  type="text"
                  id="floating_outlined"
                  className="peer block w-full appearance-none border-2 bg-transparent px-2.5 pb-2.5 pt-4 text-sm"
                  value={arg.value}
                  readOnly
                />
              </div>
            ))}
          </div>
        </div>
      </div>
      <YAMLEdit />
    </div>
  );
};

export default Launch;
