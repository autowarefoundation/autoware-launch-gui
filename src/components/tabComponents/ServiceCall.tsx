"use client";

import { useEffect, useRef, useState } from "react";
import { CheckedState } from "@radix-ui/react-checkbox";
import { invoke } from "@tauri-apps/api/tauri";
import { useAtom } from "jotai";

import { cn } from "@/lib/utils";
import {
  autowareFolderPathAtom,
  serviceListAtom,
  TopicsAndTypes,
  userEditedServiceCallFlagsAtom,
} from "@/app/jotai/atoms";

import { Button } from "../ui/button";
import { Checkbox } from "../ui/checkbox";
import { Input } from "../ui/input";
import { Label } from "../ui/label";
import { Textarea } from "../ui/textarea";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "../ui/tooltip";
import { formatForRos2Pub } from "./TopicPublish";
import { groupByPrefix } from "./TopicsBagRecord";

const EMPTY_SERVICE: TopicsAndTypes = {
  topicName: "",
  type: "",
};

const ServiceCall = () => {
  const [services, setServices] = useAtom(serviceListAtom);
  const [autowarePath, setAutowarePath] = useAtom(autowareFolderPathAtom);
  const [rate, setRate] = useAtom(userEditedServiceCallFlagsAtom);

  const [selectedService, setSelectedService] =
    useState<TopicsAndTypes>(EMPTY_SERVICE);
  const [serviceOutput, setServiceOutput] = useState<string[]>([]);
  const textAreaRef = useRef<HTMLTextAreaElement>(null);
  const outputRef = useRef<HTMLDivElement>(null);

  const handleSelectService = (topic: TopicsAndTypes) => {
    if (selectedService === topic) {
      setSelectedService(EMPTY_SERVICE);
    } else {
      setSelectedService(topic);
    }
  };

  const getServices = async () => {
    const topicsWithTypes = (await invoke("get_services", {
      payload: {},
    })) as string[];
    const topicsAndTypes = topicsWithTypes.map((topicWithType) => {
      const [topic, type] = topicWithType.split(" ");
      return { topicName: topic, type };
    });
    setServices(groupByPrefix(topicsAndTypes));
    setSelectedService(EMPTY_SERVICE);
  };

  const getMessageInterface = async (topic: TopicsAndTypes) => {
    try {
      const messageInterface = (await invoke("get_message_interface", {
        payload: {
          messageType: topic.type,
          autowarePath,
        },
      })) as string;

      const checkForEmptyObjectMessage = messageInterface.trim() === "{}";

      textAreaRef.current!.value = checkForEmptyObjectMessage
        ? `"{}"`
        : formatForRos2Pub(messageInterface);
    } catch (error) {
      console.log(error);
    }
  };

  const callService = async () => {
    const flattenedMessageWithNoExtraLines =
      textAreaRef.current!.value.replaceAll("\n", " ");

    await invoke("call_service", {
      payload: {
        serviceName: selectedService.topicName,
        serviceType: selectedService.type,
        request: flattenedMessageWithNoExtraLines,
        flag: {
          arg: rate.arg,
          value: rate.isActivated ? rate.value : "",
        },
        autowarePath,
      },
    });
    setServiceOutput(() => ["Called service: ", selectedService.topicName]);
  };

  const killService = async () => {
    await invoke("kill_service_call", {
      payload: {},
    });
    setServiceOutput((prev) => [...prev, "Killed service call"]);
  };

  useEffect(() => {
    getServices();
    async function getServiceCallOutput() {
      const { appWindow } = await import("@tauri-apps/plugin-window");

      const unlistenServiceCallOutput = await appWindow.listen<string>(
        "ros2-service-call-output",
        (output) => {
          setServiceOutput((prev) => [...prev, output.payload]);
        }
      );

      return () => {
        unlistenServiceCallOutput();
      };
    }
    getServiceCallOutput();
  }, []);

  useEffect(() => {
    // auto scroll to bottom of output container when new output is added
    if (outputRef.current) {
      outputRef.current.scrollTop = outputRef.current.scrollHeight;
    }
  }, [serviceOutput]);

  return (
    <div className="flex h-full w-full gap-4">
      <div
        id="service-list"
        className={cn(
          `flex h-full w-1/3 min-w-[33%] flex-col gap-2 overflow-auto rounded-md p-2`,
          Object.keys(services).length > 0 && `border border-input`
        )}
      >
        <div className="flex items-center gap-2">
          <Button onClick={getServices}>Refresh Service List</Button>
        </div>
        {Object.keys(services).map((prefix) => (
          <div key={prefix}>
            <h3 className="mb-2 text-lg font-bold">{prefix}</h3>
            {services[prefix].map((service) => (
              <div
                className="flex items-center gap-2 p-2"
                key={service.topicName}
              >
                <TooltipProvider>
                  <Tooltip>
                    <TooltipTrigger
                      onClick={async () => {
                        handleSelectService(service);
                        if (selectedService === service) {
                          return;
                        }
                        await getMessageInterface(service);
                      }}
                      className="flex items-center gap-2"
                    >
                      <Checkbox checked={selectedService === service} />
                      <span className="break-words font-mono text-xs">
                        {service.topicName}
                      </span>
                    </TooltipTrigger>
                    <TooltipContent>
                      <span className="font-mono text-xs">
                        Type: {service.type}
                      </span>
                    </TooltipContent>
                  </Tooltip>
                </TooltipProvider>
              </div>
            ))}
          </div>
        ))}
      </div>
      <div id="service-call" className="flex w-full max-w-[66%] flex-col gap-1">
        <Label className="text-xl">Request</Label>
        <Label className="text-xs text-muted-foreground">
          Do not remove the outermost curly braces or the quotation marks to
          make a successful service call request.
        </Label>
        <Textarea
          ref={textAreaRef}
          className=" flex-grow resize-none overflow-y-auto"
          placeholder="Enter message to send to service"
        />
        <Label className="text-xl">Output</Label>
        <div
          ref={outputRef}
          className="h-[12rem] max-h-[12rem] w-full select-text resize-none overflow-y-auto whitespace-pre-wrap break-words rounded-md border border-input p-2"
        >
          {serviceOutput.map((output, i) => (
            <div key={i}>{output}</div>
          ))}
        </div>
        <div className="flex items-center justify-end gap-2 p-2">
          <RateCheckboxInput />
          <Button
            variant={"destructive"}
            className="w-fit"
            onClick={async () => {
              await killService();
            }}
          >
            Kill Service Call
          </Button>
          <Button
            className="w-fit"
            onClick={async () => {
              await callService();
            }}
          >
            Call Service
          </Button>
        </div>
      </div>
    </div>
  );
};

export default ServiceCall;

const RateCheckboxInput = () => {
  const [rate, setRate] = useAtom(userEditedServiceCallFlagsAtom);

  const inputRef = useRef<HTMLInputElement>(null);

  const handleRateCheckboxChange = (e: CheckedState) => {
    if (typeof e !== "boolean") return;
    setRate((prev) => ({
      ...prev,
      isActivated: e,
    }));

    if (!e) {
      inputRef.current!.value = "";
      setRate(() => ({
        arg: "--rate",
        value: "",
        isActivated: false,
      }));
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (!rate.isActivated) return;
    setRate((prev) => ({
      ...prev,
      value: e.target.value,
    }));
  };

  return (
    <div className="flex items-center gap-2">
      <Label>Rate</Label>
      <Checkbox
        checked={rate.isActivated}
        onCheckedChange={(e) => handleRateCheckboxChange(e)}
      />
      <Input
        ref={inputRef}
        className="w-16"
        placeholder="10"
        onChange={handleInputChange}
        disabled={!rate.isActivated}
      />
    </div>
  );
};
