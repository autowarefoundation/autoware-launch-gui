import { useEffect, useRef, useState } from "react";
import { invoke } from "@tauri-apps/api/tauri";
import { useAtom } from "jotai";

import {
  pidsLengthAtom,
  sshCommandAtom,
  sshHostAtom,
  sshIsConnectedAtom,
  sshOutputAtom,
  sshPasswordAtom,
  sshUsernameAtom,
} from "@/app/jotai/atoms";

import { Button } from "./ui/button";
import {
  DialogContent,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "./ui/dialog";
import { Input } from "./ui/input";
import { Label } from "./ui/label";
import { useToast } from "./ui/use-toast";

export default function SSHComponent() {
  const [host, setHost] = useAtom(sshHostAtom);
  const [user, setUser] = useAtom(sshUsernameAtom);
  const [password, setPassword] = useAtom(sshPasswordAtom);
  // const [command, setCommand] = useAtom(sshCommandAtom);
  const [output, setOutput] = useAtom(sshOutputAtom);
  const [isConnected, setIsConnected] = useAtom(sshIsConnectedAtom);
  const [isLoading, setIsLoading] = useState(false);
  const outputRef = useRef<HTMLTextAreaElement>(null); // Create a ref for the textarea

  const [pidsLen, setPidsLen] = useAtom(pidsLengthAtom);

  useEffect(() => {
    // Scroll to the bottom every time the output changes
    if (outputRef.current) {
      outputRef.current.scrollTop = outputRef.current.scrollHeight;
    }
  }, [output]); // Dependency array ensures this runs only when output changes

  const { toast } = useToast();

  const connectSSH = async () => {
    try {
      setIsLoading(true);
      // First, try to establish an SSH connection
      const connectResponse = await invoke("connect_ssh", {
        payload: {
          host,
          username: user,
          password,
        },
      });

      // If the SSH connection is established, start the shell session
      if (connectResponse === "SSH connection established") {
        const shellResponse = await invoke("start_shell_session", {
          payload: {},
        });
        setIsLoading(false);
        setIsConnected(true);
        toast({
          title: "SSH Connection",
          description: "SSH connection established",
        });
      } else {
        console.error("SSH Connection Failed:", connectResponse);
        setIsConnected(false);
        toast({
          title: "SSH Connection",
          description: "SSH connection failed",
        });
      }
    } catch (error) {
      console.error(
        "Error connecting to SSH or starting shell session:",
        error
      );
      setIsConnected(false);
      toast({
        title: "SSH Connection",
        description: "SSH connection failed",
      });
    } finally {
      setIsLoading(false);
    }
  };

  // const executeSSHCommand = async () => {
  //   if (!isConnected) {
  //     console.error("Not connected to SSH");
  //     return;
  //   }
  //   try {
  //     console.log("Executing SSH command:", command);
  //     // Corrected invoke call
  //     const result = (await invoke("execute_command_in_shell", {
  //       payload: { command, user, host },
  //     })) as string;
  //     console.log("Result:", result); // This should now log the result
  //     setOutput((prevOutput) => [...prevOutput, result, "\n\n"]);
  //   } catch (error) {
  //     console.error("Error executing SSH command:", error);
  //   } finally {
  //     setCommand("");
  //     commandRef.current?.focus();
  //   }
  // };

  const disconnectSSH = async () => {
    try {
      const response = await invoke("kill_ssh_connection", { payload: {} });
      setIsConnected(false);
      toast({
        title: "SSH Connection",
        description: "SSH connection closed",
        variant: "destructive",
      });
    } catch (error) {
      console.error("Error disconnecting from SSH:", error);
    }
  };

  const clearOutput = () => {
    setOutput([]);
  };

  // const commandRef = useRef<HTMLInputElement>(null);

  return (
    <DialogContent>
      <DialogHeader>
        <DialogTitle>SSH Connection</DialogTitle>
      </DialogHeader>
      <div className="grid gap-4 py-4">
        {/* Host Input */}
        <div className="grid grid-cols-4 items-center gap-4">
          <Label htmlFor="host" className="text-left">
            Host
          </Label>
          <Input
            id="host"
            type="text"
            value={host}
            disabled={isLoading || isConnected}
            onChange={(e) => setHost(e.target.value)}
            className="col-span-3"
          />
        </div>
        {/* Username Input */}
        <div className="grid grid-cols-4 items-center gap-4">
          <Label htmlFor="user" className="text-left">
            Username
          </Label>
          <Input
            id="user"
            type="text"
            value={user}
            disabled={isLoading || isConnected}
            onChange={(e) => setUser(e.target.value)}
            className="col-span-3"
          />
        </div>
        {/* Password Input */}
        <div className="grid grid-cols-4 items-center gap-4">
          <Label htmlFor="password" className="text-left">
            Password
          </Label>
          <Input
            id="password"
            type="password"
            value={password}
            disabled={isLoading || isConnected}
            onChange={(e) => setPassword(e.target.value)}
            className="col-span-3"
          />
        </div>
        {/* Command Input */}
        {/* <div className="grid grid-cols-4 items-center gap-4">
          <Label htmlFor="command" className="text-left">
            Command
          </Label>
          <Input
            id="command"
            type="text"
            value={command}
            disabled={!isConnected}
            onChange={(e) => setCommand(e.target.value)}
            className="col-span-3"
          />
        </div> */}
      </div>
      <DialogFooter>
        <Button
          onClick={connectSSH}
          disabled={isLoading || isConnected || pidsLen > 0}
        >
          {isLoading ? "Connecting..." : "Connect"}
        </Button>
        {/* <Button onClick={executeSSHCommand} disabled={!isConnected}>
          Execute Command
        </Button> */}
        {/* <Button onClick={clearOutput} disabled={!isConnected}>
          Clear
        </Button> */}
        <Button onClick={disconnectSSH} disabled={!isConnected}>
          Disconnect
        </Button>
      </DialogFooter>
    </DialogContent>
  );
}
