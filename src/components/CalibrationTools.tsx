"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { open } from "@tauri-apps/plugin-dialog";
import { useAtom } from "jotai";

import { Button } from "@/components/ui/button";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import {
  autowareFolderPathAtom,
  calibrationToolLaunchPathsAtom,
  parsedCalibrationLaunchFilesAtom,
  selectedCalibrationLaunchArgsAtom,
  userEditedCalibrationToolArgsAtom,
} from "@/app/jotai/atoms";

import { EditArgsDialog } from "./EditArgsDialog";
import { ElementData } from "./Tree";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTrigger,
} from "./ui/dialog";
import { Input } from "./ui/input";
import { Label } from "./ui/label";
import { toast } from "./ui/use-toast";

const launchableTools = [
  "deviation_estimator",
  "deviation_evaluator",
  "deviation_evaluation_visualizer",
  "vehicle_cmd_analyzer",
  "time_delay_estimator",
  "tunable_static_tf_broadcaster",
  "extrinsic_calibration_manager",
];

const showToast = (description: string): void => {
  toast({
    type: "foreground",
    description,
    variant: "destructive",
  });
};

const CalibrationTools = () => {
  const triggerRef = useRef<HTMLButtonElement>(null);
  const dialogTriggerRef = useRef<HTMLButtonElement>(null);

  const [userEditedArgs, _setUserEditedArgs] = useAtom(
    userEditedCalibrationToolArgsAtom
  );
  const [autowarePath, setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [parsedFilePath, setParsedFilePath] = useState<string>("");

  const [elements, setElements] = useAtom(parsedCalibrationLaunchFilesAtom);
  const [selectedTool, setSelectedTool] = useState<string>("");
  const [log, setLog] = useState<string[]>([]);

  // this state needs to be in the type of an object because
  // there can be multiple tools running at the same time
  // and we need to keep track of which tool is running
  // and which tool is not
  // this is to be able to see the output of the tool that is running
  // and to be able to launch another tool
  // without having to kill the tool that is running
  const [toolLogs, setToolLogs] = useState<{ [key: string]: string[] }>({});

  const [isToolRunning, setIsToolRunning] = useState<string[]>([]);

  const [calibrationToolPaths, setCalibrationToolPaths] = useAtom(
    calibrationToolLaunchPathsAtom
  );
  const [selectedArgs, setSelectedArgs] = useAtom(
    selectedCalibrationLaunchArgsAtom
  );
  const logDivRef = useRef<HTMLDivElement>(null);

  type ToolArgs = {
    arg: string;
    value: string;
  };

  const commandGeneratorForToolLaunching = (
    tool: string,
    toolPath: string,
    userEditedArgsForThisTool: ToolArgs[] = []
  ): string => {
    if (!autowarePath) {
      showToast("Please set the Autoware path first");
      return "";
    }

    const launchFilePath = toolPath;
    if (!launchFilePath) {
      showToast("Please select a launch file first");
      return "";
    }

    const launchFile = launchFilePath.split("/").pop();
    if (!launchFile) {
      showToast("Please select a launch file first");
      return "";
    }

    let command: string;
    const commonTools = [
      "deviation_estimator",
      "deviation_evaluator",
      "time_delay_estimator",
      "tunable_static_tf_broadcaster",
      "extrinsic_calibration_manager",
    ];

    if (commonTools.includes(tool)) {
      command = `ros2 launch ${tool} ${launchFile}`;
    } else if (tool === "deviation_evaluation_visualizer") {
      const pathToParsedFileParentFolder = parsedFilePath
        .split("/")
        .slice(0, -2)
        .join("/");
      command = `pip3 install -r ${pathToParsedFileParentFolder}/requirements.txt && ros2 launch ${tool} ${launchFile}`;
    } else if (tool === "vehicle_cmd_analyzer") {
      command = `ros2 launch ${tool} ${launchFile} & ros2 run plotjuggler plotjuggler`;
    } else {
      showToast(`Unsupported tool: ${tool}`);
      return "";
    }

    const parameters = userEditedArgsForThisTool
      .map((arg) => `${arg.arg}:=${arg.value}`)
      .join(" ");
    command += ` ${parameters}`;

    return command;
  };

  // get all the arguments from the tree
  const args: {
    arg: string;
    value: string;
  }[] = [];

  const walkTreeItems = (items: ElementData[] | ElementData) => {
    if (items instanceof Array) {
      for (let i = 0; i < items.length; i++) {
        if (items[i]!.name === "arg" || items[i]!.name === "param") {
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
    async function getLaunchFiles() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      const unlistenCalibrationLaunchFileParsed = await appWindow.listen(
        "receiveTreeCalibration",
        (msg) => {
          setElements(
            JSON.parse(msg.payload as string).elements as ElementData[]
          );
        }
      );

      const unlistenCalibrationToolLaunchOutput = await appWindow.listen(
        "calibration-tool-output",
        (msg) => {
          const { tool, output } = msg.payload as {
            tool: string;
            output: string;
          };
          // set in to the log array but only if it is not already there to prevent duplicates
          setToolLogs((prev) => {
            if (!prev[tool]?.includes(output)) {
              return {
                ...prev,
                [tool]: [...(prev[tool] || []), output],
              };
            }
            return prev;
          });
        }
      );

      return () => {
        unlistenCalibrationLaunchFileParsed();
        unlistenCalibrationToolLaunchOutput();
      };
    }

    getLaunchFiles();
  }, []);

  const handleParseLaunchFile = useCallback(
    async (tool: string) => {
      if (isToolRunning.includes(tool)) {
        setParsedFilePath(calibrationToolPaths[tool]);
        setSelectedTool(tool);

        await invoke("parse_and_send_xml", {
          payload: {
            path: calibrationToolPaths[tool],
            calibrationTool: true,
          },
        });

        dialogTriggerRef.current?.click();
        return;
      }
      const file = await open({
        // xml files
        defaultPath:
          `${autowarePath}/src/autoware/calibration_tools` || undefined,
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
          description: "No launch file selected",
          variant: "destructive",
        });
      }
      if (!file) {
        return;
      }
      await invoke("parse_and_send_xml", {
        payload: {
          path: file.path,
          calibrationTool: true,
        },
      });

      setParsedFilePath(file.path);
      setCalibrationToolPaths((prev) => {
        return {
          ...prev,
          [tool]: file.path,
        };
      });
      dialogTriggerRef.current?.click();

      // setUserEditedArgs([]);
    },
    [autowarePath]
  );

  const handleArgSelectChange = useCallback(
    (arg: { arg: string; value: string }) => {
      if (!selectedTool) return; // Ensure selectedTool is defined

      // Update selectedArgs
      const updatedArgs = selectedArgs.some(
        (selectedArg) => selectedArg.arg === arg.arg
      )
        ? selectedArgs.map((selectedArg) =>
            selectedArg.arg === arg.arg ? arg : selectedArg
          )
        : [...selectedArgs, arg];

      // Merge userEditedArgs with updatedArgs
      const toolArgs = userEditedArgs[selectedTool] || [];
      const mergedArgs = toolArgs.map(
        (toolArg) =>
          updatedArgs.find((updatedArg) => updatedArg.arg === toolArg.arg) ||
          toolArg
      );

      setSelectedArgs(updatedArgs);
    },
    [selectedArgs, userEditedArgs, selectedTool]
  );

  const handleArgRemove = useCallback(
    (arg: { arg: string; value: string }) => {
      if (!selectedTool) return; // Ensure selectedTool is defined

      // Update selectedArgs
      const updatedArgs = selectedArgs.filter(
        (selectedArg) => selectedArg.arg !== arg.arg
      );

      // Merge userEditedArgs with updatedArgs
      const toolArgs = userEditedArgs[selectedTool] || [];
      const mergedArgs = toolArgs.filter((toolArg) =>
        updatedArgs.some((updatedArg) => updatedArg.arg === toolArg.arg)
      );

      setSelectedArgs(updatedArgs);
    },
    [selectedArgs, userEditedArgs, selectedTool]
  );

  const handleLaunchTool = useCallback(
    async (tool: string, toolPath: string) => {
      const commandToBeLaunched = commandGeneratorForToolLaunching(
        tool,
        toolPath,
        userEditedArgs[tool]
      );

      if (!commandToBeLaunched)
        return toast({
          type: "foreground",
          variant: "destructive",
          description: "No command to be launched, please check the arguments",
        });

      const res = await invoke("launch_tool", {
        payload: {
          tool,
          command: commandToBeLaunched,
          autowarePath,
        },
      });

      setIsToolRunning((prev) => {
        // check if the tool is already running
        // if not, add it
        // this is to prevent duplicate tools
        if (!prev.includes(tool)) return [...prev, tool];
        return prev;
      });
    },
    [userEditedArgs]
  );

  const handleKillTool = useCallback(async (tool: string) => {
    setLog((prev) => [...prev, "Killing the tool"]);
    const res = await invoke("kill_calibration_tool", {
      payload: {
        tool,
      },
    });

    setIsToolRunning((prev) =>
      prev.filter((runningTool) => runningTool !== tool)
    );

    setToolLogs((prev) => {
      const newToolLogs = { ...prev };
      delete newToolLogs[tool];
      return newToolLogs;
    });
  }, []);

  useEffect(() => {
    if (logDivRef.current) {
      logDivRef.current.scrollTop = logDivRef.current.scrollHeight;
    }
  }, [log]);

  return (
    <>
      <Popover>
        <PopoverTrigger ref={triggerRef}></PopoverTrigger>
        <Button
          onClick={async () => {
            triggerRef.current?.click();
          }}
        >
          Calibration Tools
        </Button>
        <PopoverContent className=" mt-6 flex h-60 flex-col gap-4 overflow-y-scroll">
          {launchableTools.map((tool) => (
            <Label
              className="cursor-pointer rounded-md px-2 py-4 hover:bg-primary hover:text-primary-foreground"
              key={tool}
              onClick={async () => {
                setSelectedTool(tool);
                if (isToolRunning.includes(tool)) {
                  return dialogTriggerRef.current?.click();
                }
                await handleParseLaunchFile(tool);
              }}
            >
              {tool}
            </Label>
          ))}
        </PopoverContent>
      </Popover>

      <Dialog
        modal={true}
        onOpenChange={(open) => {
          if (!open) {
            // setSelectedTool("");
          }
        }}
      >
        <DialogTrigger ref={dialogTriggerRef} className="hidden" />

        <DialogContent className="flex h-[40rem] min-w-max flex-col gap-4">
          <DialogHeader className="flex items-center justify-between">
            <Button
              onClick={async () => {
                if (!isToolRunning.includes(selectedTool)) {
                  await handleLaunchTool(
                    selectedTool,
                    calibrationToolPaths[selectedTool]
                  );
                } else {
                  await handleKillTool(selectedTool);
                }
              }}
              variant={
                isToolRunning.includes(selectedTool) ? "destructive" : "default"
              }
            >
              {`${
                isToolRunning.includes(selectedTool) ? "Kill" : "Launch"
              } ${selectedTool}`}
            </Button>
          </DialogHeader>

          {isToolRunning.includes(selectedTool) ? (
            <div className="flex h-[30rem] w-[40rem] flex-col gap-4 rounded-lg p-2">
              <span className="font-semibold">Output</span>
              <div
                className="flex h-full w-full flex-col gap-2 overflow-y-auto rounded-lg border px-2 py-4"
                ref={logDivRef}
              >
                {toolLogs?.[selectedTool]?.map((log, idx) => (
                  <Label
                    key={log + idx}
                    id="floating_outlined"
                    className="break-words border-b p-2 font-mono font-semibold"
                  >
                    {log}
                  </Label>
                ))}
              </div>
            </div>
          ) : (
            args.length > 0 && (
              <div className="flex h-[30rem] w-[40rem] items-center justify-between gap-4">
                <div className="flex h-full w-full flex-col gap-2 overflow-y-auto rounded-lg p-2">
                  <h1 className="text-lg font-semibold">
                    {parsedFilePath.split("/").pop()?.split(".")[0]} parameters
                  </h1>
                  <div className="flex h-full w-full flex-col gap-2 overflow-y-auto rounded-lg border p-2">
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
                          calibrationTool={true}
                          selectedCalibrationTool={selectedTool}
                          handleArgSelect={handleArgSelectChange}
                          handleArgRemove={handleArgRemove}
                          arg={arg}
                        />
                      </div>
                    ))}
                  </div>
                </div>
                <div className="flex h-full  w-full flex-col gap-2 overflow-y-auto rounded-lg p-2">
                  <span className="font-semibold">
                    Arguments added to launch command
                  </span>
                  {userEditedArgs[selectedTool]?.map((arg, idx) => (
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
            )
          )}
        </DialogContent>
      </Dialog>
    </>
  );
};

export default CalibrationTools;
