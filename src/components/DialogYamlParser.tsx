"use client";

import { useCallback, useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";

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
import { toast } from "./ui/use-toast";

interface YamlArgsDialogProps {
  yamlData: { [key: string]: any };
  setConfigPath: React.Dispatch<React.SetStateAction<string>>;
  path: string;
}

interface ValueObject {
  type: "string" | "integer" | "float" | "boolean";
  value: any; // You can refine this type based on your actual value structure
}
interface InitialValues {
  input: { [key: string]: string };
  checkbox: { [key: string]: boolean };
  integer: { [key: string]: number };
  double: { [key: string]: number };
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
    console.log("Yaml data changed", yamlData["/**"]["ros__parameters"]);
  }, [yamlData]);

  const isArrayRepresentation = (obj: any): obj is ValueObject[] => {
    return (
      obj &&
      typeof obj === "object" &&
      Object.keys(obj).every((key) => !isNaN(Number(key)))
    );
  };

  const initializeValues = (params: {
    [key: string]: ValueObject | ValueObject[];
  }): InitialValues => {
    const initialValues: InitialValues = {
      input: {},
      checkbox: {},
      integer: {},
      double: {},
    };

    const processValue = (valueObj: ValueObject, fullKey: string) => {
      switch (valueObj.type) {
        case "string":
          initialValues.input[fullKey] = valueObj.value;
          break;
        case "integer":
          initialValues.integer[fullKey] = valueObj.value;
          break;
        case "float":
          initialValues.double[fullKey] = valueObj.value;
          break;
        case "boolean":
          initialValues.checkbox[fullKey] = valueObj.value;
          break;
        // Add more cases as needed
      }
    };

    const traverseParams = (
      currentParams: { [key: string]: any },
      prefix: string = ""
    ) => {
      Object.entries(currentParams).forEach(([key, valueObj]) => {
        const fullKey = prefix ? `${prefix}.${key}` : key;

        if (isArrayRepresentation(valueObj)) {
          valueObj.forEach((item, index) => {
            const arrayKey = `${fullKey}.${index}`;
            if (typeof item === "object" && item !== null) {
              traverseParams({ [arrayKey]: item }, arrayKey);
            } else {
              processValue(item, arrayKey);
            }
          });
        } else if (typeof valueObj === "object" && valueObj !== null) {
          // Enhanced handling for nested objects
          if (
            valueObj.hasOwnProperty("type") &&
            valueObj.hasOwnProperty("value")
          ) {
            processValue(valueObj, fullKey);
          } else {
            Object.entries(valueObj).forEach(([nestedKey, nestedValue]) => {
              traverseParams({ [nestedKey]: nestedValue }, fullKey);
            });
          }
        }
      });
    };

    traverseParams(params);
    return initialValues;
  };

  const {
    input: initialInputValues,
    checkbox: initialCheckboxValues,
    integer: initialIntegerValues,
    double: initialDoubleValues,
  } = initializeValues(params);

  const [inputValues, setInputValues] = useState<{ [key: string]: string }>(
    initialInputValues
  );
  const [checkboxValues, setCheckboxValues] = useState<{
    [key: string]: boolean;
  }>(initialCheckboxValues);

  const [integerValues, setIntegerValues] = useState<{
    [key: string]: number;
  }>(initialIntegerValues);

  const [doubleValues, setDoubleValues] = useState<{
    [key: string]: number;
  }>(initialDoubleValues);

  const [userEditedArgParams, setUserEditedArgParams] = useState<{
    [key: string]: any;
  }>(yamlData);

  const addItem = (fullKey: string) => {
    // prepend the key with /**.ros__parameters
    const absFullKey = `/**.ros__parameters.${fullKey}`;
    const keys = absFullKey.split(".");
    let current = yamlData;

    // Check if the last part of the key is a number
    const lastKey = keys[keys.length - 1];
    if (!isNaN(Number(lastKey))) {
      // Remove the last key since it's a number
      keys.pop();
    }

    for (let i = 0; i < keys.length; i++) {
      if (i === keys.length - 1) {
        // We're at the target key
        if (Array.isArray(current[keys[i]]) && current[keys[i]].length > 0) {
          const firstItem = current[keys[i]][0];
          let newItem;

          switch (firstItem.type) {
            case "integer":
              newItem = { type: "integer", value: 0 };
              break;
            case "float":
              newItem = { type: "float", value: 0.0 };
              break;
            case "boolean":
              newItem = { type: "boolean", value: false };
              break;
            case "string":
            default:
              newItem = { type: "string", value: "" };
              break;
          }

          current[keys[i]].push(newItem);
        } else {
          console.error(`Key path ${fullKey} is not an array or is empty.`);
        }
      } else {
        // Navigate to the next level
        current = current[keys[i]];
        if (!current) {
          console.error(
            `Intermediate key path ${keys
              .slice(0, i + 1)
              .join(".")} does not exist in the YAML data.`
          );
          return;
        }
      }
    }

    setYamlData({ ...yamlData });
  };

  const removeItem = async (fullKey: string, index: number) => {
    // prepend the key with /**.ros__parameters
    const absFullKey = `/**.ros__parameters.${fullKey}`;
    const keys = absFullKey.split(".");
    let current = yamlData;

    // Check if the last part of the key is a number
    const lastKey = keys[keys.length - 1];

    if (!isNaN(Number(lastKey))) {
      // Remove the last key since it's a number
      keys.pop();
    }

    for (let i = 0; i < keys.length - 1; i++) {
      if (!current[keys[i]]) {
        console.log(
          `Key path ${keys.slice(0, i + 1).join(".")} does not exist.`
        );
        return; // If the key doesn't exist, exit early
      }
      current = current[keys[i]];
    }

    // remove item from current
    current[keys[keys.length - 1]].splice(index, 1);

    setYamlData({ ...yamlData });

    let updatedArgs = { ...userEditedArgParams };

    // Iterate over all keys to ensure format consistency
    Object.entries(updatedArgs["/**"].ros__parameters).forEach(
      ([key, value]: any) => {
        if (Array.isArray(value)) {
          value.forEach((item, index) => {
            if (
              item.type === "float" &&
              item.value % 1 === 0 &&
              !String(item.value).endsWith(".0")
            ) {
              updatedArgs["/**"].ros__parameters[key][
                index
              ].value = `${item.value}.0`;
            }
          });
        } else if (
          value.type === "float" &&
          value.value % 1 === 0 &&
          !String(value.value).endsWith(".0")
        ) {
          updatedArgs["/**"].ros__parameters[key].value = `${value.value}.0`;
        }
      }
    );

    // Create a new object with just the values
    const updatedValuesOnly = Object.fromEntries(
      Object.entries(updatedArgs["/**"].ros__parameters).map(
        ([key, value]: any) => {
          if (Array.isArray(value)) {
            return [key, value.map((item) => item.value)];
          }
          return [key, value.value];
        }
      )
    );

    updatedArgs["/**"].ros__parameters = updatedValuesOnly;

    // Convert the userEditedArgParams to an array format for further processing
    const updatedArgsArr = Object.keys(updatedArgs).map((key) => {
      return { arg: key, value: updatedArgs[key] };
    });

    // call the save_edits_yaml function
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
  };

  const handleEdit = useCallback(
    async (key: string) => {
      const basePath = "/**.ros__parameters";
      const fullKeyPath = `${basePath}.${key}`;

      const determineValue = (type: any, key: any) => {
        switch (type) {
          case "string":
            return { type, value: inputValues[key] };
          case "integer":
            return { type, value: integerValues[key] };
          case "float":
            const floatValue = doubleValues[key];
            return {
              type,
              value: floatValue % 1 === 0 ? `${floatValue}.0` : floatValue,
            };
          case "boolean":
            return {
              type,
              value:
                checkboxValues[key] !== undefined ? checkboxValues[key] : false,
            };
          default:
            return null;
        }
      };

      let valueToUpdate;
      const arrayKeyMatch = key.match(/^(\w+)\.(\d+)$/);

      if (arrayKeyMatch) {
        const [, argName, index] = arrayKeyMatch;
        const elementTypeInfo =
          yamlData["/**"]["ros__parameters"][argName][index]?.type;
        valueToUpdate = determineValue(elementTypeInfo, key);
      } else {
        const typeInfo = yamlData["/**"]["ros__parameters"][key]?.type;
        valueToUpdate = determineValue(typeInfo, key);
      }

      if (
        !valueToUpdate ||
        valueToUpdate.value == null ||
        valueToUpdate.value === ""
      ) {
        toast({
          type: "foreground",
          description: "Invalid value please check the type",
          variant: "destructive",
        });
        return;
      }

      let updatedArgs = { ...userEditedArgParams };
      updatedArgs = setNestedProperty(updatedArgs, fullKeyPath, valueToUpdate);

      // Iterate over all keys
      Object.entries(updatedArgs["/**"].ros__parameters).forEach(
        ([key, value]: any) => {
          // Check if the value is an array
          if (Array.isArray(value)) {
            value.forEach((item, index) => {
              if (
                item.type === "float" &&
                item.value % 1 === 0 &&
                !String(item.value).endsWith(".0")
              ) {
                updatedArgs["/**"].ros__parameters[key][
                  index
                ].value = `${item.value}.0`;
              }
            });
          } else if (
            value.type === "float" &&
            value.value % 1 === 0 &&
            !String(value.value).endsWith(".0")
          ) {
            updatedArgs["/**"].ros__parameters[key].value = `${value.value}.0`;
          }
        }
      );

      // Create a new object with just the values
      const updatedValuesOnly = Object.fromEntries(
        Object.entries(updatedArgs["/**"].ros__parameters).map(
          ([key, value]: any) => {
            if (Array.isArray(value)) {
              return [key, value.map((item) => item.value)];
            }
            return [key, value.value];
          }
        )
      );

      updatedArgs["/**"].ros__parameters = updatedValuesOnly;

      setUserEditedArgParams(updatedArgs);

      // Convert the userEditedArgParams to an array format for further processing
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

      setConfigPath("");
      setTimeout(() => {
        setConfigPath(path);
      }, 500);
    },
    [
      inputValues,
      checkboxValues,
      integerValues,
      doubleValues,
      path,
      setConfigPath,
      userEditedArgParams,
      yamlData,
    ]
  );

  useEffect(() => {
    const {
      input: updatedInputValues,
      checkbox: updatedCheckboxValues,
      integer: updatedIntegerValues,
      double: updatedDoubleValues,
    } = initializeValues(params);

    setInputValues(updatedInputValues);
    setCheckboxValues(updatedCheckboxValues);
    setIntegerValues(updatedIntegerValues);
    setDoubleValues(updatedDoubleValues);
  }, [yamlData]);

  const renderComponentBasedOnType = (fullKey: string, valueObj: any) => {
    switch (valueObj.type) {
      case "string":
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
      case "integer":
        return (
          <Input
            id={fullKey}
            defaultValue={integerValues[fullKey]?.toString() || ""}
            onChange={(e) => {
              setIntegerValues((prev) => ({
                ...prev,
                [fullKey]: parseInt(e.target.value, 10),
              }));
            }}
          />
        );
      case "float":
        return (
          <Input
            id={fullKey}
            defaultValue={doubleValues[fullKey]?.toString() || ""}
            onChange={(e) => {
              setDoubleValues((prev) => ({
                ...prev,
                [fullKey]: parseFloat(e.target.value),
              }));
            }}
          />
        );
      case "boolean":
        const isChecked = checkboxValues[fullKey];

        return (
          <div className="flex items-center gap-2">
            <Checkbox
              id={fullKey}
              defaultChecked={isChecked || false}
              className="col-span-3"
              onCheckedChange={(value) =>
                setCheckboxValues((prev) => ({
                  ...prev,
                  [fullKey]: value as boolean,
                }))
              }
            />
            <Label htmlFor={fullKey} className="capitalize">
              {isChecked !== undefined ? isChecked.toString() : ""}
            </Label>
          </div>
        );
      default:
        return null;
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
      if (
        typeof params[key] === "object" &&
        params[key] !== null &&
        // this object isn't a {type: "string", value: "some string"} object
        !params[key].type &&
        !params[key].value
      ) {
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
