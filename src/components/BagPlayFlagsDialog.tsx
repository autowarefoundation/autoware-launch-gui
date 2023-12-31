"use client";

import { useRef, useState } from "react";
import { useAtom } from "jotai";

import { Button } from "@/components/ui/button";
import { Checkbox } from "@/components/ui/checkbox";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { rosbagPlayFlags, userEditedBagPlayFlagsAtom } from "@/app/jotai/atoms";

import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "./ui/tooltip";

export function RosbagPlayFlagsDialog() {
  const [flags, setFlags] = useAtom(userEditedBagPlayFlagsAtom);
  // Initialize localFlags with values from flags or default to rosbagPlayFlags
  const initialLocalFlags = Object.entries(rosbagPlayFlags).reduce<{
    [key: string]: any;
  }>((acc, [key, details]) => {
    const currentFlagValue = flags.find((flag) => flag.arg === key)?.value;
    acc[key] = {
      ...details,
      value: currentFlagValue !== undefined ? currentFlagValue : details.value,
    };
    return acc;
  }, {});

  const [localFlags, setLocalFlags] = useState(initialLocalFlags);

  const handleSave = () => {
    const newFlags = Object.entries(localFlags)
      .map(([key, value]) => {
        if (value.value !== undefined)
          return {
            arg: key,
            value: value.value,
          };
      })
      .filter((flag) => flag !== undefined) as {
      arg: string;
      value: boolean;
    }[];

    setFlags(newFlags);
  };

  const triggerRef = useRef<HTMLButtonElement>(null);

  return (
    <Dialog>
      <Button onClick={() => triggerRef.current?.click()}>
        Set Rosbag Play Flags
      </Button>
      <DialogTrigger ref={triggerRef} />
      <DialogContent className="sm:max-w-[720px]">
        <DialogHeader>
          <DialogTitle>Set Rosbag Play Flags</DialogTitle>
          <DialogDescription>
            Configure the flags for the rosbag play. Click save when you're
            done. Hover over the flag name to see the description.
          </DialogDescription>
        </DialogHeader>
        <div className="grid max-h-60 gap-4 overflow-y-auto py-4">
          {Object.entries(rosbagPlayFlags).map(([flagKey, flagDetails]) => {
            const localFlagValue = localFlags[flagKey].value;
            if (typeof flagDetails.defaultValue === "boolean") {
              return (
                <div className="flex items-center gap-4" key={flagKey}>
                  <Label htmlFor={flagKey} className="w-1/6">
                    <TooltipProvider>
                      <Tooltip>
                        <TooltipTrigger className="text-left">
                          {flagKey}
                        </TooltipTrigger>
                        <TooltipContent>
                          {flagDetails.description}
                        </TooltipContent>
                      </Tooltip>
                    </TooltipProvider>
                  </Label>
                  <div className="flex w-full flex-1 gap-2">
                    <Checkbox
                      id={flagKey}
                      onCheckedChange={() => {
                        setLocalFlags((prev) => ({
                          ...prev,
                          [flagKey]: {
                            ...prev[flagKey],
                            value: !localFlagValue,
                          },
                        }));
                      }}
                      checked={(localFlagValue as boolean) ?? false}
                    />
                    <span className="font-mono text-xs">
                      {localFlagValue?.toString()}
                    </span>
                  </div>
                </div>
              );
            } else {
              return (
                <div className="flex items-center gap-4" key={flagKey}>
                  <Label htmlFor={flagKey} className="w-1/5">
                    <TooltipProvider>
                      <Tooltip>
                        <TooltipTrigger className="text-left">
                          {flagKey}
                        </TooltipTrigger>
                        <TooltipContent>
                          {flagDetails.description}
                        </TooltipContent>
                      </Tooltip>
                    </TooltipProvider>
                  </Label>
                  <Input
                    id={flagKey}
                    type="text"
                    onChange={(e) =>
                      setLocalFlags((prev) => ({
                        ...prev,
                        [flagKey]: {
                          ...prev[flagKey],
                          value: e.target.value,
                        },
                      }))
                    }
                    defaultValue={
                      (flags.find((flag) => flag.arg === flagKey)
                        ?.value as string) ??
                      localFlags[flagKey].value?.toString() ??
                      flagDetails.defaultValue.toString()
                    }
                    className=" w-full"
                    placeholder={`${flagDetails.example}`}
                  />
                </div>
              );
            }
          })}
        </div>
        <DialogFooter>
          <Button
            onClick={() => {
              triggerRef.current?.click();
              handleSave();
            }}
          >
            Save changes
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

export default RosbagPlayFlagsDialog;
