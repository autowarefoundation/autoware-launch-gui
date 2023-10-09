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
import { userEditedArgsAtom } from "@/app/jotai/atoms";

interface EditArgsDialogProps extends React.HTMLAttributes<HTMLDivElement> {
  arg: { arg: string; value: string };
  handleArgSelect: (arg: { arg: string; value: string }) => void;
  handleArgRemove: (arg: { arg: string; value: string }) => void;
}

export function EditArgsDialog(props: EditArgsDialogProps) {
  const { className, arg, handleArgSelect, handleArgRemove, ...otherProps } =
    props;
  const [userEditedArgs, setUserEditedArgs] = useAtom(userEditedArgsAtom);
  const inputRef = useRef<HTMLInputElement>(null);
  const closeRef = useRef<HTMLButtonElement>(null);
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
                id="username"
                readOnly
                defaultValue={
                  arg.value ||
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
              id="username"
              defaultValue={
                userEditedArgs.find((a) => a.arg === arg.arg)?.value || ""
              }
              className="col-span-3"
            />
          </div>
        </div>
        <DialogFooter>
          <Button
            onClick={() => {
              setUserEditedArgs((prev) =>
                prev.filter((a) => a.arg !== arg.arg)
              );
              handleArgRemove(arg);

              closeRef.current?.click();
            }}
            variant="destructive"
          >
            {userEditedArgs.find((a) => a.arg === arg.arg)
              ? "Remove from command"
              : "Cancel"}
          </Button>
          <DialogTrigger asChild>
            <Button
              onClick={() => {
                const newEditedArgs = (
                  prev: { arg: string; value: string }[]
                ) => {
                  if (!inputRef.current?.value) return prev;
                  const newArgs = prev.filter((a) => a.arg !== arg.arg);
                  return [
                    ...newArgs,
                    { arg: arg.arg, value: inputRef.current.value },
                  ];
                };

                if (inputRef.current?.value === "") {
                  setUserEditedArgs((prev) =>
                    prev.filter((a) => a.arg !== arg.arg)
                  );
                  handleArgRemove(arg);
                  return;
                }

                setUserEditedArgs((prev) => {
                  return newEditedArgs(prev);
                });

                handleArgSelect({
                  arg: arg.arg,
                  value: inputRef.current?.value || "",
                });
              }}
              variant="outline"
            >
              {userEditedArgs.find((a) => a.arg === arg.arg)
                ? "Update"
                : "Add to command"}
            </Button>
          </DialogTrigger>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
