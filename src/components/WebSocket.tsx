"use client";

import { useEffect, useState } from "react";
import { useAtom } from "jotai";

import {
  autowareFolderPathAtom,
  bagFileAtom,
  bagFileInfoAtom,
  installedPackagesAtom,
  isBagPlayingAtom,
  lastSavedLoadedProfileJSONPathsAtom,
  parsedConfigFolderStructureAtom,
  parsedLaunchFilePathAtom,
  parsedLaunchFilesAtom,
  userEditedArgsAtom,
  userEditedBagPlayFlagsAtom,
  userEditedBagRecordFlagsAtom,
  userEditedServiceCallFlagsAtom,
  userEditedTopicPubFlagsAtom,
} from "@/app/jotai/atoms";

const isWindow = typeof window !== "undefined";

const WebSocketComponent = () => {
  const [autowareFolderPath, setAutowareFolderPath] = useAtom(
    autowareFolderPathAtom
  );

  const [socket, setSocket] = useState<WebSocket | null>(null);
  const [bagFileInfo, setBagFileInfo] = useAtom(bagFileInfoAtom);
  const [bagFile, setBagFile] = useAtom(bagFileAtom);
  const [installedPackages, setInstalledPackages] = useAtom(
    installedPackagesAtom
  );
  const [isBagPlaying, setIsBagPlaying] = useAtom(isBagPlayingAtom);
  const [lastSavedLoadedProfileJSONPaths, setLastSavedLoadedProfileJSONPaths] =
    useAtom(lastSavedLoadedProfileJSONPathsAtom);
  const [parsedLaunchFilePath, setParsedLaunchFilePath] = useAtom(
    parsedLaunchFilePathAtom
  );
  const [parsedLaunchFiles, setParsedLaunchFiles] = useAtom(
    parsedLaunchFilesAtom
  );
  const [userEditedArgs, setUserEditedArgs] = useAtom(userEditedArgsAtom);
  const [userEditedBagPlayFlags, setUserEditedBagPlayFlags] = useAtom(
    userEditedBagPlayFlagsAtom
  );
  const [userEditedBagRecordFlags, setUserEditedBagRecordFlags] = useAtom(
    userEditedBagRecordFlagsAtom
  );
  const [userEditedServiceCallFlags, setUserEditedServiceCallFlags] = useAtom(
    userEditedServiceCallFlagsAtom
  );
  const [userEditedTopicPublishFlags, setUserEditedTopicPublishFlags] = useAtom(
    userEditedTopicPubFlagsAtom
  );
  const [parsedConfigFolderStructure, setParsedConfigFolderStructure] = useAtom(
    parsedConfigFolderStructureAtom
  );
  useEffect(() => {
    let ws: WebSocket;
    if (isWindow) {
      ws = new WebSocket("ws://localhost:42068/ws");
      ws.onopen = () => {
        console.log("connected to app/browser syncing websocket");

        // @ts-ignore
        if (window.__TAURI__) {
          ws.send("Tauri-WebSocket");
        } else {
          ws.send("Browser-WebSocket");
        }
      };
      ws.onclose = () => {
        console.log("connection to app/browser syncing websocket closed");
      };
      ws.onerror = (e) => {
        console.log("error from websocket", e);
      };

      setSocket(ws);
    }

    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, []);

  /**
   * localStorage items that are needed to be sent to the localhost server to sync the browser with the desktop app
   *
   * 1. autowareFolderPathLaunchGUI
   * 2. bagFileInfoLaunchGUI
   * 3. bagFileLaunchGUI
   * 4. installedPackagesLaunchGUI
   * 5. isBagPlayingLaunchGUI
   * 6. lastSavedLoadedProfileJSONPathsLaunchGUI
   * 7. parsedLaunchFilePathLaunchGUI
   * 8. parsedLaunchFilesLaunchGUI
   * 9. userEditedArgsLaunchGUI
   * 10. userEditedBagPlayFlagsLaunchGUI
   * 11. userEditedBagRecordFlagsLaunchGUI
   * 12. userEditedServiceCallFlagsLaunchGUI
   * 13. userEditedTopicPubFlagsLaunchGUI
   *
   * the localhost server is only available when the built desktop app is running and the server is started on port 42069
   * and as such we should be able to send the data to the server at the /api/sync endpoint
   * as soon as we confirm that at least one of the above items is not null which means that the user has interacted with the app
   * and that the frontend has some data that needs to be synced with the localhost server
   */

  const handleMessageComingFromTauriToBrowser = ({
    parsedReceivedMessage,
    ws,
  }: {
    parsedReceivedMessage: {
      name: MessageSenders;
      message: string;
    };
    ws: WebSocket;
  }) => {
    const parsedMessage: {
      autowareFolderPath: string | null;
      bagFileInfo: typeof bagFileInfo;
      bagFile: typeof bagFile;
      installedPackages: typeof installedPackages;
      isBagPlaying: typeof isBagPlaying;
      lastSavedLoadedProfileJSONPaths: typeof lastSavedLoadedProfileJSONPaths;
      parsedLaunchFilePath: typeof parsedLaunchFilePath;
      parsedLaunchFiles: typeof parsedLaunchFiles;
      userEditedArgs: typeof userEditedArgs;
      userEditedBagPlayFlags: typeof userEditedBagPlayFlags;
      userEditedBagRecordFlags: typeof userEditedBagRecordFlags;
      userEditedServiceCallFlags: typeof userEditedServiceCallFlags;
      userEditedTopicPublishFlags: typeof userEditedTopicPublishFlags;
      parsedConfigFolderStructure: typeof parsedConfigFolderStructure;
    } = JSON.parse(parsedReceivedMessage.message);

    // We are in the browser and we have received a message from the Tauri app

    if (parsedMessage.autowareFolderPath)
      setAutowareFolderPath(parsedMessage.autowareFolderPath);

    if (parsedMessage.bagFileInfo) setBagFileInfo(parsedMessage.bagFileInfo);

    if (parsedMessage.bagFile) setBagFile(parsedMessage.bagFile);

    if (
      parsedMessage.installedPackages &&
      parsedMessage.installedPackages.length > 0
    )
      setInstalledPackages(parsedMessage.installedPackages);

    if (parsedMessage.isBagPlaying) setIsBagPlaying(parsedMessage.isBagPlaying);

    if (
      parsedMessage.lastSavedLoadedProfileJSONPaths &&
      parsedMessage.lastSavedLoadedProfileJSONPaths.length > 0
    )
      setLastSavedLoadedProfileJSONPaths(
        parsedMessage.lastSavedLoadedProfileJSONPaths
      );

    if (parsedMessage.parsedLaunchFilePath)
      setParsedLaunchFilePath(parsedMessage.parsedLaunchFilePath);

    if (
      parsedMessage.parsedLaunchFiles &&
      parsedMessage.parsedLaunchFiles.length > 0
    )
      setParsedLaunchFiles(parsedMessage.parsedLaunchFiles);

    if (parsedMessage.userEditedArgs && parsedMessage.userEditedArgs.length > 0)
      setUserEditedArgs(parsedMessage.userEditedArgs);

    if (
      parsedMessage.userEditedBagPlayFlags &&
      parsedMessage.userEditedBagPlayFlags.length > 0
    )
      setUserEditedBagPlayFlags(parsedMessage.userEditedBagPlayFlags);

    if (
      parsedMessage.userEditedBagRecordFlags &&
      parsedMessage.userEditedBagRecordFlags.length > 0
    )
      setUserEditedBagRecordFlags(parsedMessage.userEditedBagRecordFlags);

    if (parsedMessage.userEditedServiceCallFlags)
      setUserEditedServiceCallFlags(parsedMessage.userEditedServiceCallFlags);

    if (
      parsedMessage.userEditedTopicPublishFlags &&
      parsedMessage.userEditedTopicPublishFlags.length > 0
    )
      setUserEditedTopicPublishFlags(parsedMessage.userEditedTopicPublishFlags);

    if (
      parsedMessage.parsedConfigFolderStructure &&
      Object.keys(parsedMessage.parsedConfigFolderStructure).length > 0
    )
      setParsedConfigFolderStructure(parsedMessage.parsedConfigFolderStructure);
  };

  const handleMessageComingFromBrowserToTauri = ({
    parsedReceivedMessage,
    ws,
  }: {
    parsedReceivedMessage: {
      name: MessageSenders;
      message: string;
    };
    ws: WebSocket;
  }) => {
    const parsedMessage: {
      autowareFolderPath: string | null;
      bagFileInfo: typeof bagFileInfo;
      bagFile: typeof bagFile;
      installedPackages: typeof installedPackages;
      isBagPlaying: typeof isBagPlaying;
      lastSavedLoadedProfileJSONPaths: typeof lastSavedLoadedProfileJSONPaths;
      parsedLaunchFilePath: typeof parsedLaunchFilePath;
      parsedLaunchFiles: typeof parsedLaunchFiles;
      userEditedArgs: typeof userEditedArgs;
      userEditedBagPlayFlags: typeof userEditedBagPlayFlags;
      userEditedBagRecordFlags: typeof userEditedBagRecordFlags;
      userEditedServiceCallFlags: typeof userEditedServiceCallFlags;
      userEditedTopicPublishFlags: typeof userEditedTopicPublishFlags;
      parsedConfigFolderStructure: typeof parsedConfigFolderStructure;
    } = JSON.parse(parsedReceivedMessage.message);

    console.log(parsedMessage);

    // We are in the Tauri app and we have received a message from the browser

    ws.send(JSON.stringify({ autowareFolderPath }));

    ws.send(JSON.stringify({ bagFileInfo }));

    ws.send(JSON.stringify({ bagFile }));

    ws.send(JSON.stringify({ installedPackages }));

    ws.send(JSON.stringify({ isBagPlaying }));

    ws.send(
      JSON.stringify({
        lastSavedLoadedProfileJSONPaths,
      })
    );

    ws.send(JSON.stringify({ parsedLaunchFilePath }));

    ws.send(JSON.stringify({ parsedLaunchFiles }));

    ws.send(JSON.stringify({ userEditedArgs }));

    ws.send(JSON.stringify({ userEditedBagPlayFlags }));

    ws.send(JSON.stringify({ userEditedBagRecordFlags }));

    ws.send(
      JSON.stringify({
        userEditedServiceCallFlags,
      })
    );

    ws.send(
      JSON.stringify({
        userEditedTopicPublishFlags,
      })
    );
    ws.send(JSON.stringify({ parsedConfigFolderStructure }));
  };

  const onSocketSyncRequest = (e: MessageEvent, ws: WebSocket) => {
    if (isJSONParsable(e.data)) {
      //  we then check if the parsed JSON object has a message property
      const parsedData: {
        name: MessageSenders;
        message: string;
      } = JSON.parse(e.data);

      if (parsedData.message) {
        //  if it does we then check if the name is "Tauri" then it has been sent to our localhost and if the message is json parsable
        // switch (parsedData.name) {
        // case "Tauri-WebSocket":
        if (
          parsedData.name === "Tauri-WebSocket" &&
          isJSONParsable(parsedData.message) &&
          // @ts-ignore
          !window.__TAURI__
        ) {
          handleMessageComingFromTauriToBrowser({
            parsedReceivedMessage: parsedData,
            ws,
          });
        }
        // break;
        // case "Browser-WebSocket":
        // @ts-ignore
        if (isJSONParsable(parsedData.message) && window.__TAURI__) {
          handleMessageComingFromBrowserToTauri({
            parsedReceivedMessage: parsedData,
            ws,
          });
        }
        // break;
        // default:
        // break;
        // }
      }
    }
  };

  useEffect(() => {
    if (socket && socket.readyState !== socket.CONNECTING) {
      socket.send(
        JSON.stringify({
          autowareFolderPath,
          bagFileInfo,
          bagFile,
          installedPackages,
          isBagPlaying,
          lastSavedLoadedProfileJSONPaths,
          parsedLaunchFilePath,
          parsedLaunchFiles,
          userEditedArgs,
          userEditedBagPlayFlags,
          userEditedBagRecordFlags,
          userEditedServiceCallFlags,
          userEditedTopicPublishFlags,
          parsedConfigFolderStructure,
        })
      );

      socket.onmessage = (e) => {
        onSocketSyncRequest(e, socket);
      };
    }
  }, [
    socket,
    autowareFolderPath,
    bagFileInfo,
    bagFile,
    installedPackages,
    isBagPlaying,
    lastSavedLoadedProfileJSONPaths,
    parsedLaunchFilePath,
    parsedLaunchFiles,
    userEditedArgs,
    userEditedBagPlayFlags,
    userEditedBagRecordFlags,
    userEditedServiceCallFlags,
    userEditedTopicPublishFlags,
    parsedConfigFolderStructure,
  ]);
  return <></>;
};

export default WebSocketComponent;

export function isJSONParsable(str: string): boolean {
  try {
    JSON.parse(str);
    return true;
  } catch (e) {
    return false;
  }
}

type MessageSenders = "Tauri-WebSocket" | "Browser-WebSocket";
