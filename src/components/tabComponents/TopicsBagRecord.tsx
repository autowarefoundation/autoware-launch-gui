"use client";

import { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api";
import { useAtom } from "jotai";

import { cn } from "@/lib/utils";
import type { GroupedTopics, TopicsAndTypes } from "@/app/jotai/atoms";
import {
  autowareFolderPathAtom,
  isBagRecordingAtom,
  multipleWorkspacePathsAtom,
  selectedTopicsAtom,
  topicEchoAtom,
  topicListAtom,
  userEditedBagRecordFlagsAtom,
} from "@/app/jotai/atoms";

import RosbagRecordFlagsDialog from "../BagRecordFlagsDialog";
import BagRecordLogDialog from "../BagRecordLogDialog";
import { Button } from "../ui/button";
import { Checkbox } from "../ui/checkbox";
import { Textarea } from "../ui/textarea";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "../ui/tooltip";
import { useToast } from "../ui/use-toast";

export const groupByPrefix = (topics: TopicsAndTypes[]) => {
  const grouped: GroupedTopics = {};

  topics.forEach((topic) => {
    const prefix = topic.topicName.split("/")[1];
    if (!grouped[prefix]) {
      grouped[prefix] = [];
    }
    grouped[prefix].push(topic);
  });

  return grouped;
};

const TopicsBagRecord = () => {
  const [topics, setTopics] = useAtom(topicListAtom);
  const [selectedTopics, setSelectedTopics] = useAtom(selectedTopicsAtom);
  const [isRecording, setIsRecording] = useAtom(isBagRecordingAtom);
  const [isEchoing, setIsEchoing] = useState(false);
  const [topicEcho, setTopicEcho] = useAtom(topicEchoAtom);
  const [bagRecordLog, setBagRecordLog] = useState<string[]>([]);
  const [autowarePath, setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [extraWorkspacePaths, setExtraWorkspacePaths] = useAtom(
    multipleWorkspacePathsAtom
  );
  const [userEditedBagRecordFlags, setUserEditedBagRecordFlags] = useAtom(
    userEditedBagRecordFlagsAtom
  );

  const { toast } = useToast();

  const getTopics = async () => {
    const topicsWithTypes = (await invoke("get_topics", {
      payload: {},
    })) as string[];
    const topicsAndTypes = topicsWithTypes.map((topicWithType) => {
      const [topic, type] = topicWithType.split(" ");
      return { topicName: topic, type };
    });
    setTopics(groupByPrefix(topicsAndTypes));
    setSelectedTopics([]);
  };

  const handleSelectTopic = (topic: TopicsAndTypes) => {
    if (selectedTopics.includes(topic)) {
      setSelectedTopics((prev) => prev.filter((t) => t !== topic));
    } else {
      setSelectedTopics((prev) => [...prev, topic]);
    }
  };

  useEffect(() => {
    async function init() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      const unlistenTopicEcho = await appWindow.listen(
        "ros2-topic-echo-output",
        (data) => {
          const echoOutput = data.payload as string;
          if (!isEchoing && echoOutput !== "") setIsEchoing(true);
          setTopicEcho((prev) => {
            if (prev.length > 100) {
              return [...prev.slice(50), echoOutput];
            } else {
              return [...prev, echoOutput];
            }
          });
        }
      );

      const unlistenBagRecord = await appWindow.listen(
        "ros2-bag-record-output",
        (data) => {
          const bagRecordOutput = data.payload as string;

          setBagRecordLog((prev) => {
            if (prev.length > 100) {
              return [...prev.slice(50), bagRecordOutput];
            } else {
              return [...prev, bagRecordOutput];
            }
          });
        }
      );

      const unlistenEchoKilled = await appWindow.listen(
        "topic-echo-killed",
        () => {
          setIsEchoing(false);
          toast({
            title: "Topic Echo Killed",
            description: "Topic Echo has been killed.",
            variant: "destructive",
          });
        }
      );

      const unlistenBagRecordKilled = await appWindow.listen(
        "bag-record-killed",
        () => {
          setIsRecording(false);
          toast({
            title: "Bag Record Killed",
            description: "Bag Record has been killed.",
            variant: "destructive",
          });
        }
      );

      await getTopics();
      return () => {
        unlistenTopicEcho();
        unlistenBagRecord();
        unlistenEchoKilled();
        unlistenBagRecordKilled();
      };
    }
    init();
  }, []);

  const handleTopicEcho = async () => {
    if (selectedTopics.length === 1) {
      await invoke("start_topic_echo", {
        payload: {
          topic: selectedTopics[0].topicName,
          topicType: selectedTopics[0].type,
          autowarePath,
          extraWorkspaces: extraWorkspacePaths,
        },
      });

      setTopicEcho([]);
    }
  };

  const handleKillEcho = async () => {
    await invoke("kill_topic_echo", {
      payload: {},
    });
  };

  const handleClearLog = () => {
    setTopicEcho([]);
  };

  const handleRecordBag = async () => {
    setIsRecording(true);
    await invoke("start_bag_record", {
      payload: {
        bagName: userEditedBagRecordFlags.find(
          (flag) => flag.arg === "--output"
        )?.value,
        topics: selectedTopics.map((topic) => topic.topicName),
        autowarePath,
        flags: userEditedBagRecordFlags,
        extraWorkspaces: extraWorkspacePaths,
      },
    });

    setBagRecordLog([]);
  };

  const handleStopRecording = async () => {
    setIsRecording(false);
    await invoke("kill_bag_record", {
      payload: {},
    });

    setUserEditedBagRecordFlags((prev) => {
      const outputFlag = prev.find((flag) => flag.arg === "--output");

      if (outputFlag) {
        outputFlag.value = "";
      }
      return prev;
    });
  };

  return (
    <div className="flex w-full gap-2">
      <div id="left" className="flex w-1/2 flex-col gap-2">
        <div className="flex items-center gap-2">
          <Button onClick={getTopics}>Refresh Topic List</Button>
        </div>
        <div
          className={cn(
            `flex h-full max-h-[36rem] flex-col gap-2 overflow-auto rounded-md p-2`,
            Object.keys(topics).length > 0 && `border border-input`
          )}
        >
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
                        onClick={() => handleSelectTopic(topic)}
                        className="flex items-center gap-2"
                      >
                        <Checkbox checked={selectedTopics.includes(topic)} />
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
        <div className="flex w-full gap-2">
          <BagRecordLogDialog
            handleRecordBag={handleRecordBag}
            handleStopRecording={handleStopRecording}
            bagRecordLog={bagRecordLog}
          />

          <RosbagRecordFlagsDialog />
        </div>
      </div>
      <div id="right" className="flex w-1/2 flex-col gap-2">
        <div className="flex items-center gap-2">
          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger>
                <Button
                  className="w-fit"
                  disabled={
                    selectedTopics.length === 0 ||
                    selectedTopics.length > 1 ||
                    isEchoing
                  }
                  onClick={handleTopicEcho}
                >
                  Topic Echo
                </Button>
              </TooltipTrigger>
              <TooltipContent>
                <span className="font-mono text-xs">
                  {selectedTopics.length === 0
                    ? "No topic selected"
                    : selectedTopics.length > 1
                    ? "Only one topic can be echoed at a time"
                    : isEchoing
                    ? "Topic Echo is running"
                    : "Echo the selected topic"}
                </span>
              </TooltipContent>
            </Tooltip>
          </TooltipProvider>

          <Button
            className="w-fit"
            variant="destructive"
            onClick={handleKillEcho}
            disabled={topicEcho.length === 0 || !isEchoing}
          >
            Kill Topic Echo
          </Button>
          <Button
            className="w-fit"
            variant="destructive"
            disabled={topicEcho.length === 0}
            onClick={handleClearLog}
          >
            Clear Log
          </Button>
        </div>
        <div
          className={cn(
            `flex h-full max-h-[36rem] flex-col gap-2 overflow-auto rounded-md border border-input p-2`
          )}
        >
          {/* Topic Echo TextArea */}
          <Textarea
            className="flex-1 resize-none border-none"
            placeholder="Topic Echo"
            readOnly
            value={topicEcho.join("\n")}
          />
        </div>
      </div>
    </div>
  );
};

export default TopicsBagRecord;
