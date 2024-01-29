"use client";

import { useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { useAtom } from "jotai";
import { Check, ChevronsUpDown } from "lucide-react";

import { cn } from "@/lib/utils";
import {
  Accordion,
  AccordionContent,
  AccordionItem,
  AccordionTrigger,
} from "@/components/ui/accordion";
import { Button } from "@/components/ui/button";
import { Checkbox } from "@/components/ui/checkbox";
import {
  Command,
  CommandEmpty,
  CommandGroup,
  CommandInput,
  CommandItem,
} from "@/components/ui/command";
import { Label } from "@/components/ui/label";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "@/components/ui/popover";
import { Textarea } from "@/components/ui/textarea";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import {
  autowareFolderPathAtom,
  topicListAtom,
  TopicsAndTypes,
  userEditedTopicPubFlagsAtom,
} from "@/app/jotai/atoms";

import TopicPubFlagsDialog from "../TopicPubFlagsDialog";
import { Input } from "../ui/input";
import { groupByPrefix } from "./TopicsBagRecord";

const EMPTY_TOPIC: TopicsAndTypes = {
  topicName: "",
  type: "",
};

export function formatForRos2Pub(input: string): string {
  const lines = input.split("\n");
  let result = "";
  let indentLevel = 0;
  let previousIndentLevel = 0;

  lines.forEach((line) => {
    const trimmedLine = line.trim();

    if (trimmedLine) {
      // Determine the current line's indentation
      indentLevel = line.indexOf(trimmedLine[0]);

      // Check if we are closing a bracket
      if (indentLevel < previousIndentLevel) {
        result = result.trimEnd().slice(0, -1) + "},\n";
      }

      // Add the line with necessary formatting
      if (trimmedLine.endsWith(":")) {
        result += `${" ".repeat(indentLevel)}${trimmedLine} {`;
      } else {
        result += `${" ".repeat(indentLevel)}${trimmedLine},`;
      }

      // Update previousIndentLevel for next iteration
      previousIndentLevel = indentLevel;

      result += "\n";
    }
  });

  // Close any remaining open brackets
  while (indentLevel > 0) {
    indentLevel -= 2; // Decrease indent to match opening bracket
    result = result.trimEnd().slice(0, -1) + `},\n${" ".repeat(indentLevel)}`;
  }

  // remove the last comma
  return `"{ ${result.trimEnd().slice(0, -1)} }"`;
}

const TopicPublish = () => {
  const [topics, setTopics] = useAtom(topicListAtom);
  const [autowarePath, setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [selectedTopic, setSelectedTopic] =
    useState<TopicsAndTypes>(EMPTY_TOPIC);

  const [rosMessageTypes, setRosMessageTypes] = useState<
    Array<{
      value: string;
      label: string;
    }>
  >([]);

  const [userEditedTopicPubFlags, setUserEditedTopicPubFlags] = useAtom(
    userEditedTopicPubFlagsAtom
  );
  const [publishOutput, setPublishOutput] = useState<string[]>([]);
  const textAreaRef = useRef<HTMLTextAreaElement>(null);
  const outputRef = useRef<HTMLDivElement>(null);
  const [open, setOpen] = useState(false);
  const [value, setValue] = useState("");

  const handleSelectTopic = (topic: TopicsAndTypes) => {
    if (selectedTopic === topic) {
      setSelectedTopic(EMPTY_TOPIC);
    } else {
      setSelectedTopic(topic);
    }
  };

  const getMessageTypesAvailable = async () => {
    const messageTypes = (await invoke("find_all_ros_message_types", {
      payload: {
        autowarePath,
      },
    })) as string;

    const types = messageTypes.split("\n").map((type) => type.trim());

    setRosMessageTypes(
      types.map((type) => ({
        value: type.toLowerCase(),
        label: type,
      }))
    );
  };

  const getTopics = async () => {
    const topicsWithTypes = (await invoke("get_topics", {
      payload: {},
    })) as string[];
    const topicsAndTypes = topicsWithTypes.map((topicWithType) => {
      const [topic, type] = topicWithType.split(" ");
      return { topicName: topic, type };
    });
    setTopics(groupByPrefix(topicsAndTypes));
    setSelectedTopic(EMPTY_TOPIC);
  };

  const getMessageInterface = async (messageType: string) => {
    try {
      const messageInterface = (await invoke("get_message_interface", {
        payload: {
          messageType: messageType,
          autowarePath,
        },
      })) as string;

      textAreaRef.current!.value = formatForRos2Pub(messageInterface);
    } catch (error) {
      console.log(error);
    }
  };

  const publishMessage = async () => {
    const flattenedMessageWithNoExtraLines =
      textAreaRef.current!.value.replaceAll("\n", " ");
    await invoke("publish_message", {
      payload: {
        topic: selectedTopic.topicName,
        messageType: selectedTopic.type,
        message: flattenedMessageWithNoExtraLines,
        flags: userEditedTopicPubFlags,
        autowarePath,
      },
    });

    setPublishOutput(() => [`Published to ${selectedTopic.topicName}`]);
  };

  const killPublishNode = async () => {
    await invoke("kill_topic_pub", {
      payload: {},
    });
    setPublishOutput((prev) => [...prev, `Killed topic publisher`]);
  };

  useEffect(() => {
    getTopics();
    getMessageTypesAvailable();
    async function getPublishOutput() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      const unlistenTopicPubOutput = await appWindow.listen<string>(
        "ros2-topic-pub-output",
        (output) => {
          setPublishOutput((prev) => [...prev, output.payload]);
        }
      );

      return () => {
        unlistenTopicPubOutput();
      };
    }
    getPublishOutput();
  }, []);

  // auto scroll to bottom of output container when new output is added
  useEffect(() => {
    if (outputRef.current) {
      outputRef.current.scrollTop = outputRef.current.scrollHeight;
    }
  }, [publishOutput]);

  return (
    <div className="flex h-full w-full gap-4">
      <Accordion type="single" collapsible className="w-1/3 max-w-[33%]">
        <AccordionItem value="item-1">
          <AccordionTrigger>Current Topics</AccordionTrigger>
          <AccordionContent className="flex h-full w-full flex-col gap-2">
            <div
              id="topic-list"
              className={cn(
                `flex h-full max-h-[32rem] w-full flex-col gap-2 overflow-y-auto rounded-md p-2`,
                Object.keys(topics).length > 0 && `border border-input`
              )}
            >
              <div className="flex items-center gap-2">
                <Button onClick={getTopics}>Refresh Topic List</Button>
              </div>
              {Object.keys(topics).map((prefix) => (
                <div key={prefix}>
                  <h3 className="mb-2 text-lg font-bold">{prefix}</h3>
                  {topics[prefix].map((topic) => (
                    <div
                      className="flex items-center gap-2 p-2"
                      key={topic.topicName}
                    >
                      <TooltipProvider>
                        <Tooltip>
                          <TooltipTrigger
                            onClick={async () => {
                              handleSelectTopic(topic);
                              if (selectedTopic === topic) {
                                return;
                              }
                              await getMessageInterface(topic.type);
                            }}
                            className="flex items-center gap-2"
                          >
                            <Checkbox checked={selectedTopic === topic} />
                            <span className="break-words font-mono text-xs">
                              {topic.topicName}
                            </span>
                          </TooltipTrigger>
                          <TooltipContent>
                            <span className="font-mono text-xs">
                              Type: {topic.type}
                            </span>
                          </TooltipContent>
                        </Tooltip>
                      </TooltipProvider>
                    </div>
                  ))}
                </div>
              ))}
            </div>
          </AccordionContent>
        </AccordionItem>
        <AccordionItem value="item-2">
          <AccordionTrigger>New Topic</AccordionTrigger>
          <AccordionContent className="flex h-full w-full flex-col gap-4 p-2">
            <Label className="text-xl">Topic Name</Label>
            <Input
              className="w-full"
              placeholder="Enter topic name"
              onChange={(e) => {
                setSelectedTopic({
                  topicName: e.target.value,
                  type: selectedTopic.type,
                });
              }}
            />
            <Label className="text-xl">Topic Type</Label>
            <Popover open={open} onOpenChange={setOpen}>
              <PopoverTrigger asChild>
                <Button
                  variant="outline"
                  role="combobox"
                  aria-expanded={open}
                  className="justify-between"
                >
                  <Label className="text-xs">
                    {value
                      ? rosMessageTypes.find(
                          (rosMessageType) => rosMessageType.value === value
                        )?.label
                      : "Select Message Type..."}
                  </Label>
                  <ChevronsUpDown className="ml-2 h-4 w-4 shrink-0 opacity-50" />
                </Button>
              </PopoverTrigger>
              <PopoverContent className="max-h-96 w-fit overflow-auto p-0">
                <Command>
                  <CommandInput placeholder="Search rosMessageType..." />
                  <CommandEmpty>No Message Type found.</CommandEmpty>
                  <CommandGroup>
                    {rosMessageTypes.map((rosMessageType) => (
                      <CommandItem
                        key={rosMessageType.value}
                        value={rosMessageType.value}
                        onSelect={async (currentValue) => {
                          setValue(currentValue === value ? "" : currentValue);
                          setOpen(false);
                          setSelectedTopic({
                            topicName: selectedTopic.topicName,
                            type: rosMessageType.label,
                          });

                          await getMessageInterface(rosMessageType.label);
                        }}
                        className="text-xs"
                      >
                        {/* <Check
                          className={cn(
                            "mr-2 h-4 w-4",
                            value === rosMessageType.value
                              ? "opacity-100"
                              : "opacity-0"
                          )}
                        /> */}
                        {rosMessageType.label}
                      </CommandItem>
                    ))}
                  </CommandGroup>
                </Command>
              </PopoverContent>
            </Popover>
          </AccordionContent>
        </AccordionItem>
      </Accordion>

      <div id="topic-pub" className="flex w-full max-w-[66%] flex-col gap-1">
        <Label className="text-xl">Message</Label>
        <Label className="text-xs text-muted-foreground">
          Do not remove the outermost curly braces or the quotation marks to
          make a successful publish request.
        </Label>

        <Textarea
          ref={textAreaRef}
          className="flex-grow resize-none overflow-y-auto whitespace-pre-wrap"
          placeholder="Enter message to publish"
        />
        <Label className="text-xl">Output</Label>

        <div
          ref={outputRef}
          className="h-[12rem] max-h-[12rem] w-full select-text resize-none overflow-y-auto whitespace-pre-wrap break-words rounded-md border border-input p-2"
        >
          {publishOutput.map((output, index) => (
            <div key={index}>{output}</div>
          ))}
        </div>
        <div className="flex items-center justify-end gap-2 p-2">
          <TopicPubFlagsDialog />

          <Button
            variant={"destructive"}
            className="w-fit"
            onClick={async () => {
              await killPublishNode();
            }}
          >
            Kill Node
          </Button>
          <Button
            className="w-fit"
            onClick={async () => {
              await publishMessage();
            }}
          >
            Publish
          </Button>
        </div>
      </div>
    </div>
  );
};

export default TopicPublish;
