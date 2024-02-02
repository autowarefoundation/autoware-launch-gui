"use client";

import { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import { open } from "@tauri-apps/plugin-dialog";
import { useAtom, useAtomValue } from "jotai";
import { Loader2 } from "lucide-react";
import { z } from "zod";

import { Button } from "@/components/ui/button";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import {
  autowareFolderPathAtom,
  bagFileAtom,
  bagFileInfoAtom,
  isBagPlayingAtom,
  multipleWorkspacePathsAtom,
  userEditedBagPlayFlagsAtom,
} from "@/app/jotai/atoms";

import RosbagFlagsDialog from "../BagPlayFlagsDialog";
import RosBagInfo from "../RosBagInfo";
import { Input } from "../ui/input";
import { useToast } from "../ui/use-toast";

const bagFileInfoSchema = z.object({
  bag_size: z.string(),
  duration: z.string(),
  start: z.string(),
  end: z.string(),
  messages: z.string(),
  storage_id: z.string(),
  topic_information: z.array(
    z.object({
      topic: z.string(),
      type: z.string(),
      count: z.string(),
      serialization_format: z.string(),
    })
  ),
});

export type bagFileInfoType = z.infer<typeof bagFileInfoSchema>;

// Adjust the import path based on your setup

const RosbagPlayer = () => {
  const [selectedFile, setSelectedFile] = useAtom(bagFileAtom);
  const [playbackRate, setPlaybackRate] = useState(1);
  const [bagFileInfo, setBagFileInfo] = useAtom(bagFileInfoAtom);
  const [isLoadingInfo, setIsLoadingInfo] = useState(false); // TODO: use this to show a loading spinner while waiting for bag info
  const [isPlaying, setIsPlaying] = useAtom(isBagPlayingAtom);
  const autowareFolderPath = useAtomValue(autowareFolderPathAtom);
  const [extraWorkspacePaths, setExtraWorkspacePaths] = useAtom(
    multipleWorkspacePathsAtom
  );
  const userEditedBagPlayFlags = useAtomValue(userEditedBagPlayFlagsAtom);

  const { toast } = useToast();

  const handleFileOpen = async () => {
    try {
      const file = await open({
        title: "Select a .db3 or .mcap file",
        multiple: false,
        filters: [{ name: "Rosbag Files", extensions: ["mcap", "db3"] }],
      });

      if (file && file.path.length > 0) {
        setSelectedFile(file);
        setIsLoadingInfo(true);

        const info = await invoke("get_rosbag_info", {
          path: file.path,
          autowarePath: autowareFolderPath,
          extraWorkspaces: extraWorkspacePaths,
        });

        const parsedInfo = bagFileInfoSchema.parse(info);

        setBagFileInfo(parsedInfo);
        setIsLoadingInfo(false);

        // if an old bag is already playing, stop it
        if (isPlaying) {
          await handleRosbagStop();
        }
      }
    } catch (error) {
      console.error("Error opening file:", error);
    }
  };

  const handleRosbagPlay = async () => {
    const res = await invoke("play_rosbag", {
      path: selectedFile?.path,
      autowarePath: autowareFolderPath,
      extraWorkspaces: extraWorkspacePaths,
      flags: userEditedBagPlayFlags,
    });
  };

  const handleRosbagPause = async () => {
    const res = await invoke("toggle_pause_state", {});
  };

  const handleRosbagStop = async () => {
    const res = await invoke("stop_rosbag_play", {});
    setIsPlaying(false);
  };

  const handleRosbagRate = async () => {
    const res = await invoke("set_rosbag_playback_rate", {
      rate: playbackRate,
    });
  };

  // listen to rosbag status
  useEffect(() => {
    const init = async () => {
      const unlistenStatus = await listen("rosbag-status", (data) => {
        const status = data.payload as string;
        if (status !== "unknown")
          toast({
            title: "Rosbag Status",
            description: `The rosbag is currently ${status}`,
          });
      });

      const unlistenStarted = await listen("rosbag-started", () => {
        setIsPlaying(true);

        toast({
          title: "Rosbag ready to play",
          description: "Rosbag Playback Ready",
        });
      });

      const unlistenEnded = await listen("rosbag-ended", () => {
        setIsPlaying(false);
        setPlaybackRate(1);

        toast({
          title: "Rosbag ended",
          description: "Rosbag Playback Ended",
        });
      });

      const unlistenStopped = await listen("rosbag-stopped", () => {
        setIsPlaying(false);
        setPlaybackRate(1);

        toast({
          title: "Rosbag stopped",
          description: "Rosbag Playback Stopped",
          variant: "destructive",
        });
      });

      return () => {
        unlistenStatus();
        unlistenStarted();
        unlistenEnded();
        unlistenStopped();
      };
    };
    init();
  }, []);

  return (
    <div className="flex w-full flex-col gap-4">
      <div className="grid w-full items-center gap-4">
        <div className="flex items-center gap-2">
          <Label htmlFor="rosbag file">.mcap / .db3</Label>
          {selectedFile && <Input readOnly value={selectedFile.name} />}
        </div>
        <Button onClick={handleFileOpen}>Select File</Button>

        <div className="flex w-full items-center justify-evenly gap-4">
          <Button
            onClick={async () => {
              await handleRosbagPlay();
            }}
            disabled={!selectedFile || isPlaying}
            className="w-full"
          >
            Start
          </Button>
          <Button
            onClick={async () => {
              await handleRosbagPause();
            }}
            disabled={!selectedFile || !isPlaying}
            className="w-full"
          >
            Pause/Play Toggle
          </Button>
          <Button
            onClick={async () => {
              await handleRosbagStop();
            }}
            disabled={!selectedFile || !isPlaying}
            className="w-full"
          >
            Stop
          </Button>
        </div>
        {!isPlaying ? (
          <RosbagFlagsDialog />
        ) : (
          <div className="flex items-center gap-2">
            <Slider
              value={[playbackRate]}
              min={0}
              max={20}
              step={0.1}
              onValueChange={(e) => setPlaybackRate(e[0])}
            />
            <Button
              onClick={async () => {
                await handleRosbagRate();
              }}
            >
              Set Rate to {playbackRate}
            </Button>
          </div>
        )}
      </div>

      {isLoadingInfo && (
        <div className="flex h-full items-center justify-center">
          <Loader2 className="h-20 w-20 animate-spin text-input transition-all" />
        </div>
      )}

      {bagFileInfo && <RosBagInfo data={bagFileInfo} />}
    </div>
  );
};

export default RosbagPlayer;
