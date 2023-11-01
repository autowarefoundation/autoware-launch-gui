"use client";

import type { bagFileInfoType } from "./tabComponents/RosbagPlayer";
import { Input } from "./ui/input";
import { Label } from "./ui/label";

const RosBagInfo = ({ data }: { data: bagFileInfoType }) => {
  return (
    <div className="relative max-h-full overflow-y-auto rounded-lg p-6 shadow-md">
      <h2 className="mb-4 text-2xl font-bold">Bag File Information</h2>
      <div className="mb-3">
        <Label>Bag Size:</Label>
        <Input readOnly placeholder={data.bag_size} />
      </div>
      <div className="mb-3">
        <Label>Duration:</Label>
        <Input readOnly placeholder={data.duration} />
      </div>
      <div className="mb-3">
        <Label>Start:</Label>
        <Input readOnly placeholder={data.start} />
      </div>
      <div className="mb-3">
        <Label>End:</Label>
        <Input readOnly placeholder={data.end} />
      </div>
      <div className="mb-3">
        <Label>Messages:</Label>
        <Input readOnly placeholder={data.messages} />
      </div>
      <div className="mb-3">
        <Label>Storage ID:</Label>
        <Input readOnly placeholder={data.storage_id} />
      </div>
      <h3 className="mb-4 mt-6 text-xl font-bold">Topic Information</h3>
      <ul className="max-h-72 overflow-y-auto">
        {data.topic_information.length === 0 && (
          <li className="mb-4">
            <div className="mb-2">
              <Input readOnly placeholder="No Information Available" />
            </div>
          </li>
        )}
        {data.topic_information.map((topic, index) => (
          <li key={index} className="mb-4 rounded-md border border-input p-2">
            <div className="mb-2">
              <Label>Topic:</Label>
              <Input readOnly placeholder={topic.topic} />
            </div>
            <div className="mb-2">
              <Label>Type:</Label>
              <Input readOnly placeholder={topic.type} />
            </div>
            <div className="mb-2">
              <Label>Count:</Label>
              <Input readOnly placeholder={topic.count} />
            </div>
            <div className="mb-2">
              <Label>Serialization Format:</Label>
              <Input readOnly placeholder={topic.serialization_format} />
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default RosBagInfo;
