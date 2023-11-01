"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import { useAtom } from "jotai";

import {
  isBagRecordingAtom,
  selectedTopicsAtom,
  userEditedBagRecordFlagsAtom,
} from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "./ui/dialog";

type BagRecordLogDialogProps = {
  handleRecordBag: () => Promise<void>;
  handleStopRecording: () => Promise<void>;
  bagRecordLog: string[];
};

const BagRecordLogDialog = ({
  handleRecordBag,
  bagRecordLog,
  handleStopRecording,
}: BagRecordLogDialogProps) => {
  const [selectedTopics, setSelectedTopics] = useAtom(selectedTopicsAtom);
  const [isRecording, setIsRecording] = useAtom(isBagRecordingAtom);

  const [userEditedBagRecordFlags, setUserEditedBagRecordFlags] = useAtom(
    userEditedBagRecordFlagsAtom
  );

  const triggerRef = useRef<HTMLButtonElement>(null);
  const [isOpen, setIsOpen] = useState(isRecording);

  useEffect(() => {
    setIsOpen(isRecording);
  }, [isRecording]);

  // Using useMemo to memoize the computed values
  const isOutputEmpty = useMemo(() => {
    return (
      userEditedBagRecordFlags.find((flag) => flag.arg === "--output") &&
      userEditedBagRecordFlags.find((flag) => flag.arg === "--output")
        ?.value === ""
    );
  }, [userEditedBagRecordFlags]);

  const isAllFlagSet = useMemo(() => {
    return (
      userEditedBagRecordFlags.find((flag) => flag.arg === "--all") &&
      userEditedBagRecordFlags.find((flag) => flag.arg === "--all")?.value ===
        true
    );
  }, [userEditedBagRecordFlags]);

  const areTopicsEmpty = useMemo(
    () => selectedTopics.length === 0,
    [selectedTopics]
  );

  const isDisabled = useMemo(
    () => isRecording || isOutputEmpty || (!isAllFlagSet && areTopicsEmpty),
    [isRecording, isOutputEmpty, isAllFlagSet, areTopicsEmpty]
  );

  return (
    <Dialog open={isOpen}>
      <Button
        onClick={async () => {
          triggerRef.current?.click();
          if (!isRecording) await handleRecordBag();
        }}
      >
        Record Bag
      </Button>

      <DialogTrigger ref={triggerRef} />
      <DialogContent className="sm:max-w-[720px]">
        <DialogHeader>
          <DialogTitle>Bag Recording</DialogTitle>
          <DialogDescription>
            <span className="font-mono text-xs">
              A rosbag is being recorded. Please stop the recording before you
              do anything else.
            </span>
          </DialogDescription>
        </DialogHeader>
        {/* <div className="grid h-full max-h-60 gap-4 overflow-y-auto py-4">
          {bagRecordLog.map((log, index) => (
            <span key={index} className="font-mono text-xs">
              {log}
            </span>
          ))}
        </div> */}
        <DialogFooter>
          <Button
            variant="destructive"
            onClick={async () => {
              await handleStopRecording();
              setIsOpen(false);
              triggerRef.current?.click();
            }}
          >
            Stop Recording
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
};

export default BagRecordLogDialog;
