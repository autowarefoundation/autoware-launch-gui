"use client";

import { useRef } from "react";
import { useAtom } from "jotai";

import { Button } from "@/components/ui/button";
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
import {
  userEditedArgsAtom,
  userEditedCalibrationToolArgsAtom,
} from "@/app/jotai/atoms";

interface EditArgsDialogProps extends React.HTMLAttributes<HTMLDivElement> {
  arg: { arg: string; value: string };
  handleArgSelect: (arg: { arg: string; value: string }) => void;
  handleArgRemove: (arg: { arg: string; value: string }) => void;
  calibrationTool?: boolean;
  selectedCalibrationTool?: string;
}

export function EditArgsDialog(props: EditArgsDialogProps) {
  const {
    className,
    calibrationTool,
    selectedCalibrationTool,
    arg,
    handleArgSelect,
    handleArgRemove,
    ...otherProps
  } = props;

  // Atom for general arguments
  const [userEditedArgs, setUserEditedArgs] = useAtom(userEditedArgsAtom);
  // Atom for calibration tool arguments
  const [calibrationToolArgs, setCalibrationToolArgs] = useAtom(
    userEditedCalibrationToolArgsAtom
  );

  const inputRef = useRef<HTMLInputElement>(null);
  const closeRef = useRef<HTMLButtonElement>(null);

  const updateArgs = async () => {
    const newValue = inputRef.current?.value || "";
    if (calibrationTool && selectedCalibrationTool) {
      // Update calibration tool arguments
      setCalibrationToolArgs((prevArgs) => {
        const updatedArgs = prevArgs ? { ...prevArgs } : {};
        // Ensure the array exists for the selected calibration tool
        if (!updatedArgs[selectedCalibrationTool]) {
          updatedArgs[selectedCalibrationTool] = [];
        }

        if (newValue === "") {
          updatedArgs[selectedCalibrationTool] = updatedArgs[
            selectedCalibrationTool
          ].filter((a) => a.arg !== arg.arg);
          handleArgRemove(arg);
        } else {
          const newArgs =
            updatedArgs[selectedCalibrationTool].filter(
              (a) => a.arg !== arg.arg
            ) || [];

          updatedArgs[selectedCalibrationTool] = [
            ...newArgs,
            { arg: arg.arg, value: newValue },
          ];
          handleArgSelect({ arg: arg.arg, value: newValue });
        }
        return updatedArgs;
      });
    } else {
      // Update general arguments
      if (newValue === "") {
        setUserEditedArgs((args) => args.filter((a) => a.arg !== arg.arg));
        handleArgRemove(arg);
      } else {
        setUserEditedArgs((args) => {
          const newArgs = args.filter((a) => a.arg !== arg.arg);
          return [...newArgs, { arg: arg.arg, value: newValue }];
        });
        handleArgSelect({ arg: arg.arg, value: newValue });
      }
    }

    closeRef.current?.click();
  };

  return (
    <Dialog>
      <DialogTrigger ref={closeRef} asChild>
        <Button variant="outline">Edit Parameter</Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>Edit param</DialogTitle>
          <DialogDescription>
            Edit the value of the parameter, to add to the launch command.
          </DialogDescription>
        </DialogHeader>
        <div className="grid select-none gap-4 py-4 [&>*>*]:select-none">
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="paramname" className="text-right">
              Parameter Name
            </Label>
            <Input
              id="name"
              defaultValue={arg.arg}
              className="col-span-3"
              readOnly
            />
          </div>
          {arg.value !== "no default value" && (
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="paramdefval" className="text-right">
                Parameter Default Value
              </Label>
              <Input
                ref={inputRef}
                id="defval"
                readOnly
                defaultValue={
                  arg.value ||
                  (calibrationTool &&
                    selectedCalibrationTool &&
                    calibrationToolArgs[selectedCalibrationTool]?.find(
                      (a) => a.arg === arg.arg
                    )?.value) ||
                  userEditedArgs.find((a) => a.arg === arg.arg)?.value ||
                  ""
                }
                className="col-span-3"
              />
            </div>
          )}
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="paramval" className="text-right">
              Parameter Value
            </Label>
            <Input
              ref={inputRef}
              id="newval"
              defaultValue={
                calibrationTool && selectedCalibrationTool
                  ? calibrationToolArgs[selectedCalibrationTool]?.find(
                      (a) => a.arg === arg.arg
                    )?.value
                  : userEditedArgs.find((a) => a.arg === arg.arg)?.value || ""
              }
              className="col-span-3"
            />
          </div>
        </div>
        <DialogFooter>
          <Button
            onClick={() => {
              if (calibrationTool && selectedCalibrationTool) {
                setCalibrationToolArgs((prevArgs) => {
                  const updatedArgs = prevArgs ? { ...prevArgs } : {};
                  updatedArgs[selectedCalibrationTool] = updatedArgs[
                    selectedCalibrationTool
                  ].filter((a) => a.arg !== arg.arg);
                  return updatedArgs;
                });
              } else {
                setUserEditedArgs((args) =>
                  args.filter((a) => a.arg !== arg.arg)
                );
              }
              handleArgRemove(arg);
              closeRef.current?.click();
            }}
            variant="destructive"
          >
            {calibrationTool && selectedCalibrationTool
              ? calibrationToolArgs[selectedCalibrationTool]?.find(
                  (a) => a.arg === arg.arg
                )
                ? "Remove from command"
                : "Cancel"
              : userEditedArgs.find((a) => a.arg === arg.arg)
              ? "Remove from command"
              : "Cancel"}
          </Button>
          <DialogTrigger asChild>
            <Button
              onClick={async () => {
                await updateArgs();
                closeRef.current?.click();
              }}
              variant="outline"
            >
              {calibrationTool && selectedCalibrationTool
                ? calibrationToolArgs[selectedCalibrationTool]?.find(
                    (a) => a.arg === arg.arg
                  )
                  ? "Update"
                  : "Add to command"
                : userEditedArgs.find((a) => a.arg === arg.arg)
                ? "Update"
                : "Add to command"}
            </Button>
          </DialogTrigger>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
