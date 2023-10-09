import { useCallback, useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { message } from "@tauri-apps/plugin-dialog";

import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";

import { Checkbox } from "./ui/checkbox";

interface YamlArgsDialogProps {
  yamlData: { [key: string]: any };
  setConfigPath: React.Dispatch<React.SetStateAction<string>>;
  path: string;
}

export function YamlArgsDialog({
  path,
  yamlData: initialYamlData,
  setConfigPath,
}: YamlArgsDialogProps) {
  const [yamlData, setYamlData] = useState<{ [key: string]: any }>(
    initialYamlData
  );
  const [params, setParams] = useState<{ [key: string]: any }>(
    yamlData["/**"]["ros__parameters"] || {}
  );

  useEffect(() => {
    setParams(yamlData["/**"]["ros__parameters"] || {});
  }, [yamlData]);

  const isArrayRepresentation = (obj: any) => {
    return (
      obj &&
      typeof obj === "object" &&
      Object.keys(obj).every((key) => !isNaN(Number(key)))
    );
  };

  const initializeValues = (params: { [key: string]: any }) => {
    const initialValues = {
      input: {} as { [key: string]: string },
      slider: {} as { [key: string]: number },
      checkbox: {} as { [key: string]: boolean },
    };

    const traverseParams = (
      currentParams: { [key: string]: any },
      prefix: string = ""
    ) => {
      Object.entries(currentParams).forEach(([key, value]) => {
        if (isArrayRepresentation(value)) {
          value = Object.values(value); // Convert the object to an array
        }
        const fullKey = prefix ? `${prefix}.${key}` : key;
        if (typeof value === "string") {
          initialValues.input[fullKey] = value;
        } else if (typeof value === "number") {
          initialValues.slider[fullKey] = value;
        } else if (typeof value === "boolean") {
          initialValues.checkbox[fullKey] = value;
        } else if (typeof value === "object" && value !== null) {
          traverseParams(value, fullKey);
        }
      });
    };

    traverseParams(params);

    return initialValues;
  };

  const {
    input: initialInputValues,
    slider: initialSliderValues,
    checkbox: initialCheckboxValues,
  } = initializeValues(params);

  const [inputValues, setInputValues] = useState<{ [key: string]: string }>(
    initialInputValues
  );
  const [sliderValues, setSliderValues] = useState<{ [key: string]: number }>(
    initialSliderValues
  );
  const [checkboxValues, setCheckboxValues] = useState<{
    [key: string]: boolean;
  }>(initialCheckboxValues);

  const [userEditedArgParams, setUserEditedArgParams] = useState<{
    [key: string]: any;
  }>(yamlData);

  const addItem = (fullKey: string) => {
    // prepend the key with /**.ros__parameters
    const absFullKey = `/**.ros__parameters.${fullKey}`;
    const keys = absFullKey.split(".");
    let current = yamlData;

    for (let i = 0; i < keys.length - 1; i++) {
      if (!current[keys[i]]) {
        current[keys[i]] = {};
      }
      current = current[keys[i]];

      // if the current key is an array, we push an empty string to it
      if (Array.isArray(current)) {
        current.push("");
      }
    }

    setYamlData({ ...yamlData });
  };

  const removeItem = async (fullKey: string, index: number) => {
    // prepend the key with /**.ros__parameters
    const absFullKey = `/**.ros__parameters.${fullKey}`;
    const keys = absFullKey.split(".");
    let current = yamlData;

    for (let i = 0; i < keys.length - 1; i++) {
      if (!current[keys[i]]) {
        return; // If the key doesn't exist, exit early
      }
      current = current[keys[i]];
    }

    // Check if the current key is an array and remove the item at the specified index
    if (Array.isArray(current)) {
      current.splice(index, 1);
    }

    setYamlData({ ...yamlData });

    // call the save_edits_yaml function
    await invoke("save_edits_yaml", {
      payload: {
        path,
        newYamlArgs: Object.keys(userEditedArgParams).map((key) => {
          return { arg: key, value: userEditedArgParams[key] };
        }),
      },
    });

    // Reset the config path so that the config is reloaded
    // but put it back to the original path after .5 second so that the user can continue editing
    setConfigPath("");
    setTimeout(() => {
      setConfigPath(path);
    }, 500);
  };

  const handleEdit = useCallback(
    async (key: string) => {
      const fullKeyPath = `/**.ros__parameters.${key}`;

      // Determine the value based on the key and type checks
      let valueToUpdate;
      if (
        inputValues[key] !== undefined &&
        typeof inputValues[key] === "string"
      ) {
        valueToUpdate = inputValues[key];
      } else if (
        sliderValues[key] !== undefined &&
        typeof sliderValues[key] === "number" &&
        !isNaN(sliderValues[key])
      ) {
        valueToUpdate = sliderValues[key];
      } else if (
        checkboxValues[key] !== undefined &&
        typeof checkboxValues[key] === "boolean"
      ) {
        valueToUpdate = checkboxValues[key];
      } else {
        await message(`Invalid value or type for key: ${key}}`);
        return; // Exit the function if the value or type is invalid
      }

      // Update the userEditedArgParams state using the determined value
      let updatedArgs = { ...userEditedArgParams };
      updatedArgs = setNestedProperty(updatedArgs, fullKeyPath, valueToUpdate);

      setUserEditedArgParams(updatedArgs);

      // Convert the userEditedArgParams to an array format
      const updatedArgsArr = Object.keys(updatedArgs).map((key) => {
        return { arg: key, value: updatedArgs[key] };
      });

      // Send the updated args to the Rust side
      await invoke("save_edits_yaml", {
        payload: {
          path,
          newYamlArgs: updatedArgsArr,
        },
      });

      // Reset the config path so that the config is reloaded
      // but put it back to the original path after .5 second so that the user can continue editing
      setConfigPath("");
      setTimeout(() => {
        setConfigPath(path);
      }, 500);
    },
    [
      inputValues,
      sliderValues,
      checkboxValues,
      path,
      setConfigPath,
      userEditedArgParams,
    ]
  );

  useEffect(() => {
    const {
      input: updatedInputValues,
      slider: updatedSliderValues,
      checkbox: updatedCheckboxValues,
    } = initializeValues(params);

    setInputValues(updatedInputValues);
    setSliderValues(updatedSliderValues);
    setCheckboxValues(updatedCheckboxValues);
  }, [yamlData]);

  const renderComponentBasedOnType = (fullKey: string, value: any) => {
    if (typeof value === "string") {
      return (
        <Input
          id={fullKey}
          defaultValue={inputValues[fullKey] || ""}
          className="col-span-3"
          onChange={(e) =>
            setInputValues((prev) => ({ ...prev, [fullKey]: e.target.value }))
          }
        />
      );
    } else if (typeof value === "number") {
      return (
        /*      <Slider
          id={fullKey}
          defaultValue={[sliderValues[fullKey]]}
          min={value * -10}
          max={value * 10}
          step={0.1}
          className="col-span-3"
          onValueChange={(value) =>
            setSliderValues((prev) => ({ ...prev, [fullKey]: value[0] }))
          }
        /> */
        <Input
          id={fullKey}
          defaultValue={sliderValues[fullKey] || ""}
          onChange={(e) => {
            setSliderValues((prev) => ({
              ...prev,
              [fullKey]: parseFloat(e.target.value),
            }));
          }}
        />
      );
    } else if (typeof value === "boolean") {
      return (
        <div className="flex items-center gap-2">
          <Checkbox
            id={fullKey}
            defaultChecked={checkboxValues[fullKey] || false}
            className="col-span-3"
            onCheckedChange={(value) =>
              setCheckboxValues((prev) => ({
                ...prev,
                [fullKey]: value as boolean,
              }))
            }
          />
          <Label htmlFor={fullKey} className="capitalize">
            {(checkboxValues[fullKey] as boolean).toString()}
          </Label>
        </div>
      );
    }
  };

  const renderYamlArgs = (args: { [key: string]: any }, parentKey?: string) => {
    // Check if args itself is an array representation
    if (isArrayRepresentation(args)) {
      args = Object.values(args); // Convert args to an actual array
      return args.map((value: any, index: number) => {
        const fullKey = parentKey ? `${parentKey}.${index}` : `${index}`;
        return (
          <div
            key={fullKey}
            className="flex flex-col justify-between gap-4 rounded-md border p-2"
          >
            <div className="flex items-center justify-between gap-4 pr-2">
              <Label htmlFor={fullKey} className="">
                Index: {index}
              </Label>
            </div>
            <div className="flex items-center justify-between gap-2">
              {renderComponentBasedOnType(fullKey, value)}
              <Button
                onClick={async (e) => {
                  await handleEdit(fullKey);
                }}
                variant="outline"
              >
                Save
              </Button>
              <Button
                onClick={() => {
                  removeItem(fullKey, index);
                }}
              >
                Remove
              </Button>
            </div>
            {index === args.length - 1 && (
              <Button
                onClick={() => {
                  addItem(fullKey);
                }}
              >
                Add Item
              </Button>
            )}
          </div>
        );
      });
    }

    // Continue with the existing logic for non-array representations
    return Object.keys(args).map((key) => {
      let value = args[key];
      const fullKey = parentKey ? `${parentKey}.${key}` : key;
      return (
        <div
          key={fullKey}
          className="flex flex-col justify-between gap-4 rounded-md border p-2"
        >
          <div className="flex items-center justify-between gap-4 pr-2">
            <Label htmlFor={fullKey} className="">
              {key}
            </Label>
          </div>
          <div className="flex items-center justify-between gap-2">
            {renderComponentBasedOnType(fullKey, value)}
            <Button
              onClick={async (e) => {
                e.preventDefault();
                await handleEdit(fullKey);
              }}
              variant="outline"
            >
              Save
            </Button>
          </div>
        </div>
      );
    });
  };

  const renderAllYamlArgs = () => {
    return Object.keys(params).map((key) => {
      if (typeof params[key] === "object" && params[key] !== null) {
        return (
          <div className="flex flex-col gap-2" key={key}>
            <h3 className="text-lg font-bold">{key}</h3>
            {renderYamlArgs(params[key], key)}
          </div>
        );
      } else {
        return renderYamlArgs({ [key]: params[key] });
      }
    });
  };

  return (
    <Dialog>
      <DialogTrigger asChild>
        <Button variant="outline">Edit YAML Arguments</Button>
      </DialogTrigger>
      <DialogContent
        style={{
          width: "50rem",
          maxHeight: "50rem",
        }}
      >
        <DialogHeader>
          <DialogTitle className="break-words text-center">
            {path.split("/").pop()}
          </DialogTitle>
          <DialogDescription>
            Edit the YAML arguments below. Click save when you're done.
          </DialogDescription>
        </DialogHeader>
        <div
          style={{
            width: "100%",
            maxHeight: "30rem",
            overflowY: "auto",
          }}
          className="flex flex-col gap-2 p-2"
        >
          {renderAllYamlArgs()}
        </div>
      </DialogContent>
    </Dialog>
  );
}

function setNestedProperty(obj: any, path: string, value: any) {
  const newObj = { ...obj }; // Clone the object to ensure immutability
  const keys = path.split(".");
  let current = newObj;

  for (let i = 0; i < keys.length - 1; i++) {
    if (!current[keys[i]]) {
      current[keys[i]] = {};
    }
    current = current[keys[i]];
  }

  if (Array.isArray(value)) {
    current[keys[keys.length - 1]] = [...value]; // Clone the array to ensure immutability
  } else {
    current[keys[keys.length - 1]] = value;
  }

  return newObj;
}
