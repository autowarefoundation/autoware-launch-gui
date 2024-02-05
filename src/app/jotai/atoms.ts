import { atom } from "jotai";
import { atomWithStorage } from "jotai/utils";

import { bagFileInfoType } from "@/components/tabComponents/RosbagPlayer";
import { ElementData } from "@/components/Tree";

type FileResponse = {
  base64Data?: string;
  duration?: number;
  height?: number;
  width?: number;
  mimeType?: string;
  modifiedAt?: number;
  name?: string;
  path: string;
  size: number;
};

export type TopicsAndTypes = {
  topicName: string;
  type: string;
};

export type GroupedTopics = {
  [key: string]: TopicsAndTypes[];
};

export const tabAtom = atom<
  "launch" | "rosbag" | "topics" | "publish" | "service" | "settings"
>("launch");

export const autowareFolderPathAtom = atomWithStorage<string | null>(
  "autowareFolderPathLaunchGUI",
  null
);

export const multipleWorkspacePathsAtom = atomWithStorage<Array<string>>(
  "multipleWorkspacePathsLaunchGUI",
  []
);

export const cpuUsageAtom = atom<number>(0);
export const memoryUsageAtom = atom<string>("");
export const memoryUsagePercentageAtom = atom<number>(0);
export const cpusUsageAtom = atom<number[]>([]);
export const topProcessesAtom = atom<{ name: string; cpu_usage: number }[]>([
  { name: "process1", cpu_usage: 0 },
  { name: "process2", cpu_usage: 0 },
  { name: "process3", cpu_usage: 0 },
  { name: "process4", cpu_usage: 0 },
  { name: "process5", cpu_usage: 0 },
]);

export const parsedLaunchFilesAtom = atomWithStorage<ElementData[]>(
  "parsedLaunchFilesLaunchGUI",
  []
);

export const parsedLaunchFilePathAtom = atomWithStorage<string>(
  "parsedLaunchFilePathLaunchGUI",
  ""
);

export const selectedLaunchArgsAtom = atom<{ arg: string; value: string }[]>(
  []
);

export const selectedCalibrationLaunchArgsAtom = atom<
  { arg: string; value: string }[]
>([]);

export const userEditedArgsAtom = atomWithStorage<
  { arg: string; value: string }[]
>("userEditedArgsLaunchGUI", []);

export const parsedCalibrationLaunchFilesAtom = atomWithStorage<ElementData[]>(
  "parsedCalibrationLaunchFilesLaunchGUI",
  []
);

export const calibrationToolArgsAtom = atom<{ arg: string; value: string }[]>(
  []
);

export const calibrationToolLaunchPathsAtom = atomWithStorage<{
  [calibrationTool: string]: string;
}>("calibrationToolLaunchPathsLaunchGUI", {});

export const userEditedCalibrationToolArgsAtom = atomWithStorage<{
  [calibrationTool: string]: { arg: string; value: string }[];
}>("userEditedCalibrationToolArgsLaunchGUI", {});

export const pidsLengthAtom = atom<number>(0);

export const autowareProcessesAtom = atom<string[]>([]);

export const launchLogsAllAtom = atom<string[]>([]);
export const launchLogsInfoAtom = atom<string[]>([]);
export const launchLogsWarnAtom = atom<string[]>([]);
export const launchLogsErrorAtom = atom<string[]>([]);
export const launchLogsDebugAtom = atom<string[]>([]);
export const launchLogsComponentAtom = atom<{ name: string; logs: string[] }[]>(
  []
);
export const topicEchoAtom = atom<string[]>([]);

export const installedPackagesAtom = atomWithStorage<string[]>(
  "installedPackagesLaunchGUI",
  []
);

export const lastSavedLoadedProfileJSONPathsAtom = atomWithStorage<string[]>(
  "lastSavedLoadedProfileJSONPathsLaunchGUI",
  []
);
export const lastUsedAutowareFoldersAtom = atomWithStorage<string[]>(
  "lastUsedAutowareFoldersLaunchGUI",
  []
);

export const bagFileAtom = atomWithStorage<FileResponse | null>(
  "bagFileLaunchGUI",
  null
);
export const bagFileInfoAtom = atomWithStorage<bagFileInfoType | null>(
  "bagFileInfoLaunchGUI",
  null
);

export const isBagPlayingAtom = atomWithStorage<boolean>(
  "isBagPlayingLaunchGUI",
  false
);
export const isBagRecordingAtom = atom<boolean>(false);

export const topicListAtom = atom<GroupedTopics>({});
export const serviceListAtom = atom<GroupedTopics>({});
export const selectedTopicsAtom = atom<TopicsAndTypes[]>([]);

export const userEditedBagPlayFlagsAtom = atomWithStorage<
  {
    arg: string;
    value: string | boolean | number;
  }[]
>("userEditedBagPlayFlagsLaunchGUI", []);

export const userEditedBagRecordFlagsAtom = atomWithStorage<
  {
    arg: string;
    value: string | boolean | number;
  }[]
>("userEditedBagRecordFlagsLaunchGUI", []);

export const userEditedTopicPubFlagsAtom = atomWithStorage<
  {
    arg: string;
    value: string | boolean | number;
  }[]
>("userEditedTopicPubFlagsLaunchGUI", []);

export const sshHostAtom = atom<string>("");
export const sshUsernameAtom = atom<string>("");
export const sshPasswordAtom = atom<string>("");
export const sshCommandAtom = atom<string>("");
export const sshOutputAtom = atom<string[]>([]);
export const sshIsConnectedAtom = atom<boolean>(false);

export const rosbagPlayFlags: {
  [key: string]: {
    description: string;
    example: string;
    defaultValue: string | boolean | number;
    value?: string | boolean | number;
  };
} = {
  "--storage": {
    description: "Specifies the storage implementation of the bag.",
    example: "mcap",
    defaultValue: "",
  },
  "--read-ahead-queue-size": {
    description:
      "Sets the size of the message queue rosbag tries to hold in memory.",
    example: "100",
    defaultValue: "",
  },
  "--rate": {
    description: "Sets the rate at which to play back messages.",
    example: "1.5",
    defaultValue: "",
  },
  "--topics": {
    description: "Specifies which topics to replay.",
    example: "/camera/image /lidar/pointcloud",
    defaultValue: "",
  },
  "--qos-profile-overrides-path": {
    description:
      "Path to a yaml file defining overrides of the QoS profile for specific topics.",
    example: "/path/to/qos_overrides.yaml",
    defaultValue: "",
  },
  "--loop": {
    description: "Enables loop playback.",
    example: "--loop",
    defaultValue: false,
  },
  "--remap": {
    description: "Remaps topics.",
    example: "/old_topic1:=/new_topic1 /old_topic2:=/new_topic2",
    defaultValue: "",
  },
  "--storage-config-file": {
    description:
      "Path to a yaml file defining storage specific configurations.",
    example: "/path/to/storage_config.yaml",
    defaultValue: "",
  },
  "--clock": {
    description: "Publish to /clock at a specific frequency.",
    example: "--clock",
    defaultValue: false,
  },
  "--delay": {
    description: "Sleep duration before play.",
    example: "2.5",
    defaultValue: "",
  },
  "--disable-keyboard-controls": {
    description: "Disables keyboard controls for playback.",
    example: "--disable-keyboard-controls",
    defaultValue: false,
  },
  "--start-paused": {
    description: "Start the playback player in a paused state.",
    example: "--start-paused",
    defaultValue: false,
  },
  "--start-offset": {
    description:
      "Start the playback player this many seconds into the bag file.",
    example: "5.0",
    defaultValue: "",
  },
  "--wait-for-all-acked": {
    description: "Wait until all published messages are acknowledged.",
    example: "2000",
    defaultValue: "",
  },
  "--disable-loan-message": {
    description: "Disable to publish as loaned message.",
    example: "--disable-loan-message",
    defaultValue: false,
  },
};

export const rosbagRecordFlags: {
  [key: string]: {
    description: string;
    example: string;
    defaultValue: string | boolean | number;
    value?: string | boolean | number;
  };
} = {
  "--all": {
    description:
      "Record all topics. Required if no explicit topic list or regex filters.",
    example: "--all",
    defaultValue: false,
  },
  "--regex": {
    description: "Record only topics containing provided regular expression.",
    example: "--regex REGEX",
    defaultValue: "",
  },
  "--exclude": {
    description: "Exclude topics containing provided regular expression.",
    example: "--exclude EXCLUDE",
    defaultValue: "",
  },
  "--include-unpublished-topics": {
    description: "Discover and record topics which have no publisher.",
    example: "--include-unpublished-topics",
    defaultValue: false,
  },
  "--include-hidden-topics": {
    description: "Discover and record hidden topics as well.",
    example: "--include-hidden-topics",
    defaultValue: false,
  },
  "--output": {
    description: "Destination of the bagfile to create.",
    example: "--output OUTPUT",
    defaultValue: "",
  },
  "--storage": {
    description: "Storage identifier to be used.",
    example: "--storage sqlite3",
    defaultValue: "sqlite3",
  },
  "--serialization-format": {
    description: "RMW serialization format in which the messages are saved.",
    example: "--serialization-format s",
    defaultValue: "",
  },
  "--no-discovery": {
    description: "Disables topic auto discovery during recording.",
    example: "--no-discovery",
    defaultValue: false,
  },
  "--polling-interval": {
    description:
      "Time in ms to wait between querying available topics for recording.",
    example: "--polling-interval 1000",
    defaultValue: "",
  },
  "--max-bag-size": {
    description: "Maximum size in bytes before the bagfile will be split.",
    example: "--max-bag-size 1000000",
    defaultValue: 0,
  },
  "--max-bag-duration": {
    description:
      "Maximum duration in seconds before the bagfile will be split.",
    example: "--max-bag-duration 60",
    defaultValue: 0,
  },
  "--max-cache-size": {
    description:
      "Maximum size (in bytes) of messages to hold in each buffer of cache.",
    example: "--max-cache-size 100000",
    defaultValue: "",
  },
  "--compression-mode": {
    description: "Determine whether to compress by file or message.",
    example: "--compression-mode file",
    defaultValue: "none",
  },
  "--compression-format": {
    description: "Specify the compression format/algorithm.",
    example: "--compression-format zstd",
    defaultValue: "none",
  },
  "--compression-queue-size": {
    description:
      "Number of files or messages that may be queued for compression before being dropped.",
    example: "--compression-queue-size 10",
    defaultValue: 1,
  },
  "--compression-threads": {
    description:
      "Number of files or messages that may be compressed in parallel.",
    example: "--compression-threads 4",
    defaultValue: 0,
  },
  "--snapshot-mode": {
    description: "Enable snapshot mode.",
    example: "--snapshot-mode",
    defaultValue: false,
  },
  "--ignore-leaf-topics": {
    description: "Ignore topics without a publisher.",
    example: "--ignore-leaf-topics",
    defaultValue: false,
  },
  "--qos-profile-overrides-path": {
    description:
      "Path to a yaml file defining overrides of the QoS profile for specific topics.",
    example: "--qos-profile-overrides-path /path/to/qos_overrides.yaml",
    defaultValue: "",
  },
  "--storage-preset-profile": {
    description: "Select a configuration preset for storage.",
    example: "--storage-preset-profile default",
    defaultValue: "",
  },
  "--storage-config-file": {
    description:
      "Path to a yaml file defining storage specific configurations.",
    example: "--storage-config-file /path/to/storage_config.yaml",
    defaultValue: "",
  },
  "--start-paused": {
    description: "Start the recorder in a paused state.",
    example: "--start-paused",
    defaultValue: false,
  },
  "--use-sim-time": {
    description:
      "Use simulation time for message timestamps by subscribing to the /clock topic.",
    example: "--use-sim-time",
    defaultValue: false,
  },
};

export const rosTopicPubFlags: {
  [key: string]: {
    description: string;
    example: string;
    defaultValue: string | boolean | number;
    value?: string | boolean | number;
  };
} = {
  "--rate": {
    description:
      "The publishing rate (in Hz) for continuously publishing messages.",
    example: "10",
    defaultValue: "",
  },
  "--once": {
    description: "Publish only one message, then exit.",
    example: "",
    defaultValue: false,
  },
  "--times": {
    description: "Number of times to publish the message.",
    example: "5",
    defaultValue: "",
  },
  "--print": {
    description: "Print the message to the console before publishing.",
    example: "",
    defaultValue: false,
  },
  "--keep-alive": {
    description:
      "Keep the publisher alive by spinning in the background after the last message is sent.",
    example: "",
    defaultValue: false,
  },
  "--qos-profile": {
    description: "Use a pre-configured Quality of Service profile by name.",
    example: "sensor-data",
    defaultValue: "",
  },
  "--qos-depth": {
    description:
      "Queue size setting to publish with (overrides depth value of --qos-profile option)",
    example: "10",
    defaultValue: "",
  },
  "--qos-durability": {
    description:
      "Quality of service durability setting to publish with (overrides durability value of --qos-profile option, default: transient_local)",
    example: "transient_local",
    defaultValue: "",
  },
  "--qos-reliability": {
    description:
      "Quality of service reliability setting to publish with (overrides reliability value of --qos-profile option, default: reliable)",
    example: "reliable",
    defaultValue: "",
  },
  "--qos-history": {
    description:
      "History of samples setting to publish with (overrides history value of --qos-profile option, default: keep_last)",
    example: "keep_last",
    defaultValue: "",
  },
  "--node-name": {
    description: "Name of the node to publish with.",
    example: "my_node",
    defaultValue: "",
  },
  "--wait-matching-subscriptions": {
    description:
      'Wait until finding the specified number of matching subscriptions. Defaults to 1 when using "-1"/"--once"/"--times", otherwise defaults to 0.',
    example: "1",
    defaultValue: "",
  },
  "--use-sim-time": {
    description:
      "Use simulation time for message timestamps by subscribing to the /clock topic.",
    example: "",
    defaultValue: false,
  },
  "--spin-time": {
    description:
      "Spin time in seconds to wait for discovery (only applies when not using an already running daemon).",
    example: "1.0",
    defaultValue: "",
  },
};

export const rosServiceCallFlags: {
  [key: string]: {
    description: string;
    example: string;
    defaultValue: string | boolean | number;
    value?: string | boolean | number;
  };
} = {
  "--rate": {
    description:
      "The publishing rate (in Hz) for continuously publishing messages.",
    example: "10",
    defaultValue: "",
  },
};

export const userEditedServiceCallFlagsAtom = atomWithStorage<{
  arg: string;
  value: string;
  isActivated: boolean;
}>("userEditedServiceCallFlagsLaunchGUI", {
  arg: "--rate",
  value: "",
  isActivated: false,
});
