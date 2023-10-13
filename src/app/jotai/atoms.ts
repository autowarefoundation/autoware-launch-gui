import { atom } from "jotai";
import { atomWithStorage } from "jotai/utils";

import { ElementData } from "@/components/Tree";

// export const mapScriptPathAtom = atomWithStorage<string | null>(
//   "mapScriptPath",
//   null
// );
// export const sensingScriptPathAtom = atomWithStorage<string | null>(
//   "sensingScriptPath",
//   null
// );
// export const localizationScriptPathAtom = atomWithStorage<string | null>(
//   "localizationScriptPath",
//   null
// );
// export const detectionScriptPathAtom = atomWithStorage<string | null>(
//   "detectionScriptPath",
//   null
// );
// export const missionPlanningScriptPathAtom = atomWithStorage<string | null>(
//   "missionPlanningScriptPath",
//   null
// );
// export const motionPlanningScriptPathAtom = atomWithStorage<string | null>(
//   "motionPlanningScriptPath",
//   null
// );

export const autowareFolderPathAtom = atomWithStorage<string | null>(
  "autowareFolderPathLaunchGUI",
  null
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

export const userEditedArgsAtom = atomWithStorage<
  { arg: string; value: string }[]
>("userEditedArgsLaunchGUI", []);

export const pidsLengthAtom = atom<number>(0);

export const autowareProcessesAtom = atom<string[]>([]);

export const launchLogsInfoAtom = atom<string[]>([]);
export const launchLogsWarnAtom = atom<string[]>([]);
export const launchLogsErrorAtom = atom<string[]>([]);
export const launchLogsDebugAtom = atom<string[]>([]);
export const launchLogsComponentAtom = atom<{ name: string; logs: string[] }[]>(
  []
);
export const installedPackagesAtom = atomWithStorage<string[]>(
  "installedPackagesLaunchGUI",
  []
);

export const lastSavedLoadedProfileJSONPathsAtom = atomWithStorage<string[]>(
  "lastSavedLoadedProfileJSONPathsLaunchGUI",
  []
);
