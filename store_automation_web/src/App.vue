<script setup>
import { RouterLink, RouterView } from 'vue-router'
</script>

<template

>
  <div style="widht: 100%">

  <grid-layout
    :layout.sync="layout"
    :col-num="40"
    :row-height="10"
    :is-draggable="true"
    :is-resizable="true"
    :is-mirrored="false"
    :vertical-compact="false"
    :margin="[10, 10]"
    :use-css-transforms="true"
    :preventCollision="false"
    :autoSize="false"
  >
    <grid-item v-for="item in layout"
        :x="item.x"
        :y="item.y"
        :w="item.w"
        :h="item.h"
        :i="item.i"
        :key="item.i">

      <img id="image_sub" v-if="item.i === '2'" />
      <div id="button_view" v-if="item.i === '3'">
        <table id="key_button_table">
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('load_left')"
                @mouseup="control_button_up('load_left')"
              >load left</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('unload_left')"
                @mouseup="control_button_up('unload_left')"
              >unload left</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('standby')"
                @mouseup="control_button_up('standby')"
              >standby</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('unload_right')"
                @mouseup="control_button_up('unload_right')"
              >unload right</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('load_right')"
                @mouseup="control_button_up('load_right')"
              >load right</n-button></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell" colspan="5"></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell"><n-button
                class="key_but"
                @mousedown="queue_execution = !queue_execution"
              >{{ queue_execution ? 'Pause' : 'Resume' }}</n-button></td>
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell">
              <n-button 
                class="key_but"
                @mousedown="control_button_down('up')"
                @mouseup="control_button_up('up')"
              >⇧</n-button></td>
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell"></td>
          </tr>
          <tr class="key_button_table_row" >
            <td class="key_button_table_cell"></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('left')"
                @mouseup="control_button_up('left')"
              >⇦</n-button>
            </td>
            <td class="key_button_table_cell">
              <n-button 
                class="key_but"
                @mousedown="control_button_down('down')"
                @mouseup="control_button_up('down')"
              >⇩</n-button></td>
            <td class="key_button_table_cell">
              <n-button
                class="key_but"
                @mousedown="control_button_down('right')"
                @mouseup="control_button_up('right')"
              >⇨</n-button></td>
            <td class="key_button_table_cell"></td>
          </tr>
        </table>

        
              
      </div>

      <div v-if="item.i === '0'">
        <n-form>
        <n-select
          placeholder="Select operation"
          :options="operations_select"
          v-model:value="adding_operation"
        ></n-select>
        <n-select
        v-if="adding_operation == 'goto_cell'"
          placeholder="Select cell"
          :options="cells"
          @update:value="cellUpdateValue"
        ></n-select>
        <n-input-number 
          v-if="adding_operation == 'goto_coords'"
          v-model:value="inputed_coords.x"
          placeholder="Select X"
        ></n-input-number>
        <n-input-number 
          v-if="adding_operation == 'goto_coords'"
          v-model:value="inputed_coords.y"
          placeholder="Select Y"
        ></n-input-number>
        <n-input-number 
          v-if="adding_operation == 'goto_coords'"
          v-model:value="inputed_coords.yaw"
          placeholder="Select YAW"
        ></n-input-number>
        <n-button
          v-if="adding_operation != ''"
          @mousedown="addOperationToQueue()"
        >Add operation to queue</n-button>
        </n-form>
        <n-table :bordered="true" :single-line="false">
          <thead>
            <tr>
              <th>ID</th>
              <th>Operation</th>
              <th>Coordinates</th>
              <th>Status</th>
              <th>Delete</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="(op, id, index) in operation_queue">
              <td>{{ id }}</td>
              <td>{{ op.operation }}</td>
              <td>{{ op.coordinates }}</td>
              <td>{{ op.status }}</td>
              <td><n-button 
                class="key_but"
                @mousedown="delOperationFromQueue(id)"
              >DEL</n-button></td>
            </tr>
          </tbody>
        </n-table>
          
      </div>
      

    </grid-item>
    </grid-layout>
  
  </div>

</template>

<script>
import { GridLayout, GridItem } from 'vue3-grid-layout-next';
import { NButton, NTable, NInputNumber, NForm, NSelect } from 'naive-ui'
import ros from './mixins/ros.js';
import { times } from 'lodash';


export default {
  mixins: [ros],
  components: {GridLayout, GridItem, NTable, NButton, NInputNumber, NForm,
               NSelect},
  data () {
    return {
      operations_select: [
        {label: 'goto_coords', value: "goto_coords"},
        {label: 'goto_cell', value: "goto_cell"},
        {label: 'load_left', value: "load_left"},
        {label: 'unload_left', value: "unload_left"},
        {label: 'load_right', value: "load_right"},
        {label: 'unload_right', value: "unload_right"},
        {label: 'standby', value: "standby"}
      ],
      cells: [
        {label: 'input', value: {x: 0, y: 0, yaw: 0}},
        {label: 'output', value: {x: 0, y: 0, yaw: 0}},
        {label: '0', value: {x: 0, y: 0, yaw: 0}},
        {label: '1', value: {x: 0, y: 0, yaw: 0}},
        {label: '2', value: {x: 0, y: 0, yaw: 0}},
        {label: '3', value: {x: 0, y: 0, yaw: 0}},
        {label: '4', value: {x: 0, y: 0, yaw: 0}}
      ],
      adding_operation: '',
      inputed_coords: {
        x: 6, y: 6, yaw: 1.57
      },
      operation_counter: 0,
      operation_queue: {

      },
      layout: [
        {"x":0,"y":0,"w":11,"h":23,"i":"0", static: false},
        // {"x":30,"y":0,"w":10,"h":23,"i":"1", static: false},
        {"x":27,"y":33,"w":13,"h":13,"i":"3", static: false},
      ],
      control_buttons: {
        up: false,
        down: false,
        left: false,
        right: false,
        load_left: false,
        load_right: false,
        unload_left: false,
        unload_right: false
      }
   }
  },
  mounted() {
    document.addEventListener('keydown', (e) => {
      switch (e.code) {
        case 'ArrowLeft': case 'KeyA':
          this.control_button_down('left');
          break;
        case 'ArrowRight': case 'KeyD':
          this.control_button_down('right');
          break;
        case 'ArrowUp': case 'KeyW':
          this.control_button_down('up');
          break;
        case 'ArrowDown': case 'KeyS':
          this.control_button_down('down');
          break;
        default:
          break;
      }

    });
    document.addEventListener('keyup', (e) => {
      switch (e.code) {
        case 'ArrowLeft': case 'KeyA':
          this.control_button_up('left');
          break;
        case 'ArrowRight': case 'KeyD':
          this.control_button_up('right');
          break;
        case 'ArrowUp': case 'KeyW':
          this.control_button_up('up');
          break;
        case 'ArrowDown': case 'KeyS':
          this.control_button_up('down');
          break;
        default:
          break;
      }
    });
  },
  methods: {
    delOperationFromQueue(id) {
      if (this.operation_queue[id]) {
        delete this.operation_queue[id]
      }

    },
    addOperationToQueue() {
      this.operation_counter += 1;
      this.operation_queue[this.operation_counter] = {
        operation: this.adding_operation,
        coordinates: this.adding_operation == 'goto_coords' 
                      || this.adding_operation == 'goto_cell' ? this.inputed_coords : '---',
        status: 'added'
      }

         
      this.adding_operation = ''
    },
    cellUpdateValue(value, option) {
      console.log('cell_coords: ', value)
      this.inputed_coords = value;
    },
    handleUpdateValue(value, option) {
      this.adding_operation = option.label;
    },
    control_button_down(button) {
      this.control_buttons[button] = true;
    },
    control_button_up(button) {
      this.control_buttons[button] = false;
    }
  }
}
</script>

<style scoped>
.lcol {
  height: inherit;
  border: 1px solid grey;
}
.layout {
  background-color: #ddd;
}
.test {
  background-color: #ddd;
}
.vue-grid-item {
  width: 150px;
  text-align: center;
  border: 1px solid #ddd;
  border-radius: 10px;
  /* margin: 10px 0; */
  /* padding: 10px; */
}

#image_sub {
  width: inherit;
  height: inherit;
}

.key_but {
  width: 100%;
  height: 100%;
  overflow: hidden;
  overflow-wrap: break-word;
  word-break: break-all;
}

#button_view {
  padding: 20px;
  width: 100%;
  height: 100%;
}

#key_button_table {
  width: inherit;
  height: inherit;
  
  /* border: 1px solid grey; */
}

  
/* .key_button_table_cell {
  border: 1px solid green;
} */

.light-green {
  height: auto;
  background-color: rgba(0, 128, 0, 0.12);
}
.green {
  height: auto;
  background-color: rgba(0, 128, 0, 0.24);
}
</style>
