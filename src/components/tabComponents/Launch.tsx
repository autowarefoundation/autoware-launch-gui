"use client";

import React, { useCallback, useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import { Window } from "@tauri-apps/api/window";
import { open } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  autowareProcessesAtom,
  isBagPlayingAtom,
  launchLogsAllAtom,
  launchLogsComponentAtom,
  launchLogsDebugAtom,
  launchLogsErrorAtom,
  launchLogsInfoAtom,
  launchLogsWarnAtom,
  multipleWorkspacePathsAtom,
  parsedLaunchFilePathAtom,
  parsedLaunchFilesAtom,
  pidsLengthAtom,
  selectedLaunchArgsAtom,
  sshHostAtom,
  sshIsConnectedAtom,
  sshUsernameAtom,
  userEditedArgsAtom,
} from "@/app/jotai/atoms";

import AutowareFolderSetup from "../AutowareFolderSetup";
import { AutowareLaunchDialog } from "../AutowareLaunchDialog";
import CalibrationTools from "../CalibrationTools";
import { EditArgsDialog } from "../EditArgsDialog";
import { ProfileSetup } from "../ProfileSetup";
import { type ElementData } from "../Tree";
import { Button } from "../ui/button";
import { Input } from "../ui/input";
import { Label } from "../ui/label";
import { useToast } from "../ui/use-toast";
import { isJSONParsable } from "../WebSocket";
import YAMLEdit from "./YamlEdit";

const isWindow = typeof window !== "undefined";

const Launch = () => {
  const [pidsLen, setPidsLen] = useAtom(pidsLengthAtom);
  const [elements, setElements] = useAtom(parsedLaunchFilesAtom);
  const [_launchFilePath, _setLaunchFilePath] = useAtom(
    parsedLaunchFilePathAtom
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
          ws.send("Tauri-Launch");
        } else {
          ws.send("Browser-Launch");
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
  }, []);

  const [tauriCommand, setTauriCommand] = useState("");

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
  const [extraWorkspacePaths] = useAtom(multipleWorkspacePathsAtom);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const [autowareProcessesNames, _setAutowareProcessesNames] = useAtom(
    autowareProcessesAtom
  );

  const [parsedLaunchFile, _] = useAtom(parsedLaunchFilesAtom);
  const [launchLogsAll, setLaunchLogsAll] = useAtom(launchLogsAllAtom);
  const [launchLogsInfo, setLaunchLogsInfo] = useAtom(launchLogsInfoAtom);
  const [launchLogsWarn, setLaunchLogsWarn] = useAtom(launchLogsWarnAtom);
  const [launchLogsError, setLaunchLogsError] = useAtom(launchLogsErrorAtom);
  const [launchLogsDebug, setLaunchLogsDebug] = useAtom(launchLogsDebugAtom);
  const [launchLogsComponent, setLaunchLogsComponent] = useAtom(
    launchLogsComponentAtom
  );

  const [host, _setHost] = useAtom(sshHostAtom);
  const [user, _setUser] = useAtom(sshUsernameAtom);
  const [isSSHConnected, _setIsConnected] = useAtom(sshIsConnectedAtom);
  const [, setIsBagPlaying] = useAtom(isBagPlayingAtom);
  const [autowareLaunchedInSSH, setAutowareLaunchedInSSH] = useState(false);

  // get all the arguments from the tree
  const args: {
    arg: string;
    value: string;
  }[] = [];

  const { toast } = useToast();

  useEffect(() => {
    setEditableCommand(`ros2 launch autoware_launch ${launchFilePath}`);
  }, [launchFilePath]);

  useEffect(() => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    async function init() {
      // TODO: Figure out the tauri-controls situation
      const unlistenCloseRequest = await listen(
        "close_requested",
        async (msg) => {
          await invoke("kill_autoware_process", {});
          setIsBagPlaying(false);
          await Window.getCurrent().destroy();
        },
        {
          target: "main",
        }
      );

      const unlistenPidsLen = await listen(
        "pids_len",
        (msg) => {
          if (typeof msg.payload === "number")
            setPidsLen(msg.payload as number);
        },
        {
          target: "main",
        }
      );

      const unlistenPidsCleared = await listen(
        "pids-cleared",
        (msg) => {
          setPidsLen(0);
        },
        {
          target: "main",
        }
      );

      const unlistenAutowareLaunchFileParsed = await listen(
        "receiveTree",
        (msg) => {
          setElements(
            JSON.parse(msg.payload as string).elements as ElementData[]
          );
        },
        {
          target: "main",
        }
      );

      const unlistenPackageNotFound = await listen(
        "package-not-found",
        () => {
          toast({
            title: "Caught an Exception",
            description:
              "Exception hit, please check the error logs or try running the app from the terminal `autoware-launch-gui`",
            variant: "destructive",
          });
        },
        {
          target: "main",
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

  useEffect(() => {
    // @ts-ignore
    if (!(isWindow && window.__TAURI__)) {
      return;
    }
    async function init() {
      const unlistenStartAutowareOnAppStart = await listen<boolean>(
        "start_autoware_on_app_start",
        async (msg) => {
          await handleLaunchAutoware();
        },
        {
          target: "main",
        }
      );

      const unlistenKillAutowareThroughTray = await listen<boolean>(
        "kill_autoware_through_tray",
        async (msg) => {
          await invoke("kill_autoware_process", {});
        },
        {
          target: "main",
        }
      );
      return () => {
        unlistenKillAutowareThroughTray();
        unlistenStartAutowareOnAppStart();
      };
    }
    if (autowarePath && launchFilePath) init();
  }, [autowarePath, launchFilePath]);

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

  const handleLaunchAutoware = useCallback(async () => {
    // @ts-ignore
    if (!window.__TAURI__) {
      if (socket) {
        return socket.send(
          JSON.stringify({
            typeOfMessage: "launch",
            message: { command: "launch-autoware" },
          })
        );
      } else {
        return toast({
          variant: "destructive",
          title: "Error",
          description:
            "Please check the socket connection, you might need to restart the app or refresh the page",
        });
      }
    }
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
      const mapPathContent: string[] = await invoke("files_in_dir", {
        mapPath: mapPath,
      });

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
          // @ts-ignore
          if (isWindow && window.__TAURI__) {
            await invoke("launch_autoware", {
              path: autowarePath,
              launchFile: launchFilePath,
              extraWorkspaces: extraWorkspacePaths,
              argsToLaunch,
            });
          }
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
          // @ts-ignore
          if (isWindow && window.__TAURI__) {
            await invoke("execute_command_in_shell", {
              command: cmdStr,
              user,
              host,
            });
          }
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
  }, [autowarePath, launchFilePath, parsedLaunchFile, userEditedArgs, socket]);

  const handleParseLaunchFile = useCallback(async () => {
    // @ts-ignore
    if (isWindow && !window.__TAURI__ && socket) {
      return socket.send(JSON.stringify({ command: "parse-launch-file" }));
    }
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
      path: file.path,
      calibrationTool: false,
    });

    // const fileNameWithExtension = file.path.split("/").pop();
    setParsedFilePath(file.path);
    setUserEditedArgs([]);
  }, [autowarePath, socket]);

  const handleKillAutoware = useCallback(async () => {
    if (!isSSHConnected) {
      // @ts-ignore
      if (isWindow && !window.__TAURI__) {
        if (socket) {
          socket.send(
            JSON.stringify({
              typeOfMessage: "launch",
              message: { command: "kill-autoware" },
            })
          );
        }
        // @ts-ignore
      } else if (isWindow && window.__TAURI__) {
        await invoke("kill_autoware_process", {});
      }
    } else {
      // @ts-ignore
      if (isWindow && !window.__TAURI__) {
        if (socket) {
          socket.send(
            JSON.stringify({
              typeOfMessage: "launch",
              message: { command: "kill-ssh-connection" },
            })
          );
        }
        // @ts-ignore
      } else if (isWindow && window.__TAURI__) {
        await invoke("kill_ssh_connection", {});
      }
      _setIsConnected(false);
      setAutowareLaunchedInSSH(false);

      toast({
        variant: "destructive",
        title: "Autoware Killed in Remote Machine",
        description:
          "Autoware is closed and SSH Connection Closed Successfully",
      });
    }
  }, [isSSHConnected, socket]);

  const launchButtonRef = useRef<HTMLButtonElement>(null);
  const killButtonRef = useRef<HTMLButtonElement>(null);
  const parseLaunchFileButtonRef = useRef<HTMLButtonElement>(null);

  const handleBrowserCalls = (tauriCommand: string) => {
    switch (tauriCommand) {
      case "launch-autoware":
        setTauriCommand("");
        launchButtonRef.current?.click();
        break;
      case "kill-autoware":
        setTauriCommand("");
        killButtonRef.current?.click();
        break;
      case "parse-launch-file":
        setTauriCommand("");
        parseLaunchFileButtonRef.current?.click();
        break;
      default:
        break;
    }
  };

  useEffect(() => {
    handleBrowserCalls(tauriCommand);
  }, [tauriCommand, socket]);

  const handleTauriMessages = () => {
    // @ts-ignore
    if (isWindow && window.__TAURI__) {
      if (socket) {
        socket.send(
          JSON.stringify({
            typeOfMessage: "pids_len",
            message: { pids_len: pidsLen },
          })
        );
        socket.send(
          JSON.stringify({
            typeOfMessage: "launchlogs",
            message: {
              launchLogsAll,
              launchLogsComponent,
              launchLogsDebug,
              launchLogsError,
              launchLogsInfo,
              launchLogsWarn,
            },
          })
        );
      }
      // @ts-ignore
    } else if (isWindow && !window.__TAURI__) {
      if (socket) {
        socket.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (isJSONParsable(data.message)) {
            const parsedData = JSON.parse(data.message) as {
              typeOfMessage: string;
              message: {
                pids_len: number;
                launchLogsAll: string[];
                launchLogsComponent: { name: string; logs: string[] }[];
                launchLogsDebug: string[];
                launchLogsError: string[];
                launchLogsInfo: string[];
                launchLogsWarn: string[];
              };
            };
            if (parsedData.typeOfMessage === "pids_len")
              setPidsLen(parsedData.message.pids_len);
            if (parsedData.typeOfMessage === "launchlogs") {
              setLaunchLogsAll(parsedData.message.launchLogsAll);
              setLaunchLogsComponent(parsedData.message.launchLogsComponent);
              setLaunchLogsDebug(parsedData.message.launchLogsDebug);
              setLaunchLogsError(parsedData.message.launchLogsError);
              setLaunchLogsInfo(parsedData.message.launchLogsInfo);
              setLaunchLogsWarn(parsedData.message.launchLogsWarn);
            }
          }
        };
      }
    }
  };

  const handleButtonCallsFromBrowser = () => {
    // @ts-ignore
    if (isWindow && window.__TAURI__) {
      if (socket) {
        socket.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (isJSONParsable(data.message)) {
            const parsedData = JSON.parse(data.message) as {
              typeOfMessage: string;
              message: {
                command: string;
              };
            };
            if (parsedData.typeOfMessage === "launch")
              setTauriCommand(parsedData.message.command);
          }
        };
      }
    }
  };

  useEffect(() => {
    handleButtonCallsFromBrowser();
  }, [socket, tauriCommand]);

  useEffect(() => {
    handleTauriMessages();
  }, [
    pidsLen,
    launchLogsAll,
    launchLogsComponent,
    launchLogsDebug,
    launchLogsError,
    launchLogsInfo,
    launchLogsWarn,
    socket,
  ]);

  return (
    <div className="flex h-full w-full flex-col gap-4">
      <div className="flex flex-row gap-4">
        <AutowareLaunchDialog />
        <Button
          ref={parseLaunchFileButtonRef}
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
          ref={launchButtonRef}
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
          ref={killButtonRef}
          onClick={async () => {
            await handleKillAutoware();
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
